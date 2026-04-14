#include "TorqueMap.h"
#include <algorithm>
#include <cmath>

/**
 * Map a pedal travel percentage to a torque request.
 */

namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr float kSmallValue = 1.0e-6f;

float clampValue(float value, float lowerBound, float upperBound) {
    return std::max(lowerBound, std::min(value, upperBound));
}

float shapePedalRequest(float pedalFraction, float exponentialFactor) {
    float clampedPedalFraction = clampValue(pedalFraction, 0.0f, 1.0f);
    if(std::fabs(exponentialFactor) < kSmallValue) {
        return clampedPedalFraction;
    }

    float denominator = std::exp(exponentialFactor) - 1.0f;
    if(std::fabs(denominator) < kSmallValue) {
        return clampedPedalFraction;
    }

    float numerator = std::exp(exponentialFactor * clampedPedalFraction) - 1.0f;
    return clampValue(numerator / denominator, 0.0f, 1.0f);
}

} // namespace

void TorqueMap::evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime) {
    float fullPedalTorqueRequest = std::max(params->mapPedalToTorqueRequest(1.0f), 0.0f);
    float mappedPedalTorqueRequest = clampValue(params->mapPedalToTorqueRequest(input->apps), 0.0f, fullPedalTorqueRequest);
    float pedalRequestFraction = 0.0f;
    if(fullPedalTorqueRequest > kSmallValue) {
        pedalRequestFraction = mappedPedalTorqueRequest / fullPedalTorqueRequest;
    }
    pedalRequestFraction = shapePedalRequest(pedalRequestFraction, params->mapPedalExponentialFactor);

    float pedalTorqueRequest = fullPedalTorqueRequest;
    float derate = 1.0f;

//    derate = params->mapDerateMotorTemp(input->motorTemp);
//    pedalTorqueRequest *= derate;
//
//    derate = params->mapDerateInverterTemp(input->inverterTemp);
//    pedalTorqueRequest *= derate;
//
//    derate = params->mapDerateBatteryTemp(input->batteryTemp);
//    pedalTorqueRequest *= derate;
//
//    derate = params->mapDerateBatterySoc(input->batterySoc);
//    pedalTorqueRequest *= derate;
//

    currentOvershootFilter.add(input->batteryCurrent, deltaTime);
    if(std::fabs(input->batteryCurrent) < 1.0f) {
        openCircuitVoltageFilter.add(input->batteryVoltage, deltaTime);
    }
    float openCircuitVoltage = openCircuitVoltageFilter.get();

    float currentLimit = 200.0f; // Amps (reduced from 230A)
    float dischargeVoltageForPowerLimit = openCircuitVoltage;
    if(input->batteryCurrent > 2.0f) {
        dischargeVoltageForPowerLimit = std::min(openCircuitVoltage, input->batteryVoltage);
    }
    float currentBasedPowerLimit = dischargeVoltageForPowerLimit * currentLimit;
    float powerLimit = std::min(params->mapPowerLimit, currentBasedPowerLimit);
    powerLimit = std::max(powerLimit, 0.0f);

    // battery OCV based derate
    derate = clampValue((openCircuitVoltage / 128.0f - 3.4f) / 0.1f, 0.0f, 1.0f); // updated to 128s config from 126. Linear derate 3.5 to 3.4 OCV cell
    pedalTorqueRequest *= derate;

    float measuredBatteryPower = input->batteryVoltage * input->batteryCurrent;
    measuredPowerFilter.add(measuredBatteryPower, deltaTime);
    float filteredBatteryPower = measuredPowerFilter.get();
    float measuredBatteryPowerRate = 0.0f;
    if(deltaTime > 0.0f && this->hasMeasuredPowerHistory) {
        measuredBatteryPowerRate = (measuredBatteryPower - this->previousMeasuredBatteryPower) / deltaTime;
    }
    this->previousMeasuredBatteryPower = measuredBatteryPower;
    this->hasMeasuredPowerHistory = true;

    float motorRpmMagnitude = std::fabs(input->motorRpm);
    float efficiency = clampValue(params->mapPowerLimitMotorEfficiency(motorRpmMagnitude), 0.50f, 1.00f);
    float limitedMotorRpm = std::max(motorRpmMagnitude, std::max(params->mapPowerLimitMinRpm, 1.0f));
    float motorAngularVelocity = limitedMotorRpm * kTwoPi / 60.0f;
    float mechanicalPowerLimit = powerLimit * efficiency;
    float feedforwardTorque = 0.0f;
    if(motorAngularVelocity > 0.0f) {
        feedforwardTorque = mechanicalPowerLimit / motorAngularVelocity;
    }
    feedforwardTorque = clampValue(feedforwardTorque, 0.0f, pedalTorqueRequest);

    float smoothedCurrent = currentOvershootFilter.get();
    if(smoothedCurrent > 240.0f || measuredBatteryPower > 85000.0f) {
        this->integral = 0.0f;
        output->torqueRequest = 0.0f;
        output->ocvEstimate = openCircuitVoltage;
        output->powerLimit = powerLimit;
        output->feedbackP = 0.0f;
        output->feedbackI = 0.0f;
        output->feedbackD = 0.0f;
        output->feedbackTorque = 0.0f;
        return;
    }

    float powerError = powerLimit - filteredBatteryPower;

    if(input->apps < 0.01f || pedalTorqueRequest <= 0.0f) {
        this->integral = 0.0f;
    }

    float proportional = params->mapPowerLimit_kP * powerError;
    float candidateIntegral = this->integral;
    if(deltaTime > 0.0f) {
        candidateIntegral += powerError * deltaTime;
    }
    float derivative = -params->mapPowerLimit_kD * std::max(measuredBatteryPowerRate, 0.0f);
    float trimLimit = std::max(params->mapPowerLimitTrimLimit, 0.0f);
    float feedbackMin = -trimLimit;
    float feedbackMax = trimLimit;

    float candidateIntegralTerm = params->mapPowerLimit_kI * candidateIntegral;
    float unsaturatedTrim = proportional + candidateIntegralTerm + derivative;

    bool saturatingHigh = unsaturatedTrim > feedbackMax && powerError > 0.0f;
    bool saturatingLow = unsaturatedTrim < feedbackMin && powerError < 0.0f;
    if(!(saturatingHigh || saturatingLow)) {
        this->integral = candidateIntegral;
    }

    float integralTrim = params->mapPowerLimit_kI * this->integral;
    float trimTorque = proportional + integralTrim + derivative;
    trimTorque = clampValue(trimTorque, feedbackMin, feedbackMax);

    float availableTorque = feedforwardTorque + trimTorque;
    availableTorque = clampValue(availableTorque, 0.0f, pedalTorqueRequest);

    float torqueRequest = pedalRequestFraction * availableTorque;
    torqueRequest = clampValue(torqueRequest, 0.0f, pedalTorqueRequest);

    output->torqueRequest = torqueRequest;

    output->ocvEstimate = openCircuitVoltage;
    output->powerLimit = powerLimit;
    output->feedbackP = pedalRequestFraction * proportional;
    output->feedbackI = pedalRequestFraction * integralTrim;
    output->feedbackD = pedalRequestFraction * derivative;
    output->feedbackTorque = pedalRequestFraction * trimTorque;
}

void TorqueMap::setParameters(VcuParameters *params) {
    this->measuredPowerFilter = LowPassFilter(std::max(params->mapPowerLimitMeasuredPowerLpfTimeConstant, 0.0f));
    this->measuredPowerFilter.reset();
    this->integral = 0.0f;
    this->previousMeasuredBatteryPower = 0.0f;
    this->hasMeasuredPowerHistory = false;
}
