#include "TorqueMap.h"

/**
 * Map a pedal travel percentage to a torque request.
 */

void TorqueMap::evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime) {
    float torqueRequest = params->mapPedalToTorqueRequest(input->apps);
    float derate;

//    derate = params->mapDerateMotorTemp(input->motorTemp);
//    torqueRequest *= derate;
//
//    derate = params->mapDerateInverterTemp(input->inverterTemp);
//    torqueRequest *= derate;
//
//    derate = params->mapDerateBatteryTemp(input->batteryTemp);
//    torqueRequest *= derate;
//
//    derate = params->mapDerateBatterySoc(input->batterySoc);
//    torqueRequest *= derate;
//

    currentOvershootFilter.add(input->batteryCurrent, deltaTime);
    if(input->batteryCurrent > -1.0f && input->batteryCurrent < 1.0f) {
        openCircuitVoltageFilter.add(input->batteryVoltage, deltaTime);
    }
    float openCircuitVoltage = openCircuitVoltageFilter.get();
    float internalResistance = 0.690; //this was 0.750 changed to account for reduced cell groups of 128. Added .05 for consistency from previous

    float currentLimit = 200.0f; // Amps (reduced from 230A)
    float currentBasedPowerLimit = (openCircuitVoltage - (currentLimit * internalResistance)) * currentLimit;
    float powerLimit = params->mapPowerLimit;
    if(currentBasedPowerLimit < powerLimit) {
        powerLimit = currentBasedPowerLimit;
    }
    if(powerLimit < 0) {
        powerLimit = 0;
    }



    // battery OCV based derate
    derate = std::max(std::min((openCircuitVoltage/128.0f - 3.5f) / 0.1f, 1.0f), 0.0f); //updated to 128s config from 126. Linear derate 3.6 to 3.5 OCV cell
    torqueRequest *= derate;

//    float motorAngularVelocity = input->motorRpm / 60.0f * 2.0f * 3.14159f; // rad/s
//    float maxTorqueAtPowerLimit = powerLimit / motorAngularVelocity * 0.90f; // Nm
//    if(torqueRequest > maxTorqueAtPowerLimit) {
//        torqueRequest = maxTorqueAtPowerLimit;
//    }

    float smoothedCurrent = currentOvershootFilter.get();
    float currentPower = input->batteryVoltage * input->batteryCurrent;
    if(smoothedCurrent > 240.0f || currentPower > 85000.0f) {
        torqueRequest = 0;
    }

    float powerError = powerLimit - currentPower;

    if(input->apps < 0.01f || torqueRequest <= 0.0f) {
        this->integral = 0.0f;
        this->prevError = powerError;
    }

    float derivativeError = 0.0f;
    if(deltaTime > 0.0f) {
        derivativeError = (powerError - this->prevError) / deltaTime;
    }

    float proportional = params->mapPowerLimit_kP * powerError;
    float candidateIntegral = this->integral;
    if(deltaTime > 0.0f) {
        candidateIntegral += powerError * deltaTime;
    }
    float derivative = params->mapPowerLimit_kD * derivativeError;

    float feedbackMin = -torqueRequest;
    float feedbackMax = 0.0f;

    float candidateIntegralTerm = params->mapPowerLimit_kI * candidateIntegral;
    float unsaturatedFeedback = proportional + candidateIntegralTerm + derivative;

    bool saturatingHigh = unsaturatedFeedback > feedbackMax && powerError > 0.0f;
    bool saturatingLow = unsaturatedFeedback < feedbackMin && powerError < 0.0f;
    if(!(saturatingHigh || saturatingLow)) {
        this->integral = candidateIntegral;
    }

    float integral = params->mapPowerLimit_kI * this->integral;
    float feedback = proportional + integral + derivative;
    feedback = std::max(feedbackMin, std::min(feedback, feedbackMax));

    torqueRequest += feedback; // feedback is negative
    if(torqueRequest < 0) {
        torqueRequest = 0;
    }

    output->torqueRequest = torqueRequest;

    output->ocvEstimate = openCircuitVoltage;
    output->powerLimit = powerLimit;
    output->feedbackP = proportional;
    output->feedbackI = integral;
    output->feedbackD = derivative;
    output->feedbackTorque = feedback;

    this->prevError = powerError;
}

void TorqueMap::setParameters(VcuParameters *params) {
    (void) params;
    this->integral = 0.0f;
    this->prevError = 0.0f;
}
