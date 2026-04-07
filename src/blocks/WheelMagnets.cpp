#include "WheelMagnets.h"

#include <cmath>

namespace {
constexpr float kPi = 3.14159265f;
constexpr float kSignalCyclesPerWheelRevolution = 3.0f;
constexpr float kFieldFilterTimeConstant = 0.010f;
constexpr float kAmplitudeFilterTimeConstant = 0.050f;
constexpr float kActivityFilterTimeConstant = 0.035f;
constexpr float kDerivativeFilterTimeConstant = 0.006f;
constexpr float kFrequencyFilterTimeConstant = 0.020f;
constexpr float kOutputFilterTimeConstant = 0.035f;
constexpr float kMinSignalAmplitude = 0.25f;
constexpr float kStoppedTimeoutMin = 0.250f;
constexpr float kStoppedActivityThreshold = 0.8f;
constexpr float kZeroSpeedThreshold = 0.1f;
constexpr float kMaxSignalOmega = 800.0f;
constexpr float kMaxWheelAcceleration = 450.0f;

float clamp(float value, float minValue, float maxValue) {
    if (value < minValue) {
        return minValue;
    }

    if (value > maxValue) {
        return maxValue;
    }

    return value;
}

void resetTracker(WheelTracker *tracker) {
    tracker->hasEstimate = false;
    tracker->hasPreviousSample = false;
    tracker->phase = 0.0f;
    tracker->previousPhase = 0.0f;
    tracker->signalOmega = 0.0f;
    tracker->omegaSample1 = 0.0f;
    tracker->omegaSample2 = 0.0f;
    tracker->wheelSpeedEstimate = 0.0f;
    tracker->previousFilteredField = 0.0f;
    tracker->amplitudeEstimate = 0.0f;
    tracker->quadratureEstimate = 0.0f;
    tracker->activityEstimate = 0.0f;
    tracker->stillTime = 0.0f;
}

void invalidateSensor(WheelTracker *tracker, LowPassFilter& fieldFilter, LowPassFilter& speedFilter, float *wheelSpeed) {
    fieldFilter.reset();
    speedFilter.reset();
    resetTracker(tracker);
    *wheelSpeed = 0.0f;
}

float lowPassStep(float previousValue, float newValue, float timeConstant, float deltaTime) {
    if (deltaTime <= 0.0f || timeConstant <= 0.0f) {
        return newValue;
    }

    float alpha = deltaTime / (timeConstant + deltaTime);
    return previousValue + ((newValue - previousValue) * alpha);
}

float wrapAngle(float angle) {
    while (angle > kPi) {
        angle -= 2.0f * kPi;
    }

    while (angle < -kPi) {
        angle += 2.0f * kPi;
    }

    return angle;
}

float median3(float a, float b, float c) {
    if (a > b) {
        float temp = a;
        a = b;
        b = temp;
    }

    if (b > c) {
        float temp = b;
        b = c;
        c = temp;
    }

    if (a > b) {
        float temp = a;
        a = b;
        b = temp;
    }

    return b;
}
}

WheelTracker dispFr{false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
WheelTracker dispFl{false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
WheelTracker dispBr{false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
WheelTracker dispBl{false, false, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void WheelMagnets::setParameters(VcuParameters *params) {
    fieldFilterFl = LowPassFilter(kFieldFilterTimeConstant);
    fieldFilterFr = LowPassFilter(kFieldFilterTimeConstant);
    fieldFilterBl = LowPassFilter(kFieldFilterTimeConstant);
    fieldFilterBr = LowPassFilter(kFieldFilterTimeConstant);
    speedFilterFl = LowPassFilter(params->wheelMagnetLpfTimeConstant);
    speedFilterFr = LowPassFilter(params->wheelMagnetLpfTimeConstant);
    speedFilterBl = LowPassFilter(params->wheelMagnetLpfTimeConstant);
    speedFilterBr = LowPassFilter(params->wheelMagnetLpfTimeConstant);
    resetTracker(&dispFl);
    resetTracker(&dispFr);
    resetTracker(&dispBl);
    resetTracker(&dispBr);
}


float calcSpeed(WheelTracker *tracker, float field, float deltaTime, LowPassFilter& fieldFilter, LowPassFilter& speedFilter) {
    fieldFilter.add(field, deltaTime);
    float filteredField = fieldFilter.get();
    tracker->amplitudeEstimate = lowPassStep(
        tracker->amplitudeEstimate,
        std::fabs(filteredField),
        kAmplitudeFilterTimeConstant,
        deltaTime
    );

    if (!tracker->hasPreviousSample) {
        tracker->hasPreviousSample = true;
        tracker->previousFilteredField = filteredField;
        return 0.0f;
    }

    float fieldSlope = (filteredField - tracker->previousFilteredField) / deltaTime;
    tracker->activityEstimate = lowPassStep(
        tracker->activityEstimate,
        std::fabs(fieldSlope),
        kActivityFilterTimeConstant,
        deltaTime
    );

    if (tracker->activityEstimate < kStoppedActivityThreshold) {
        tracker->stillTime += deltaTime;
    } else {
        tracker->stillTime = 0.0f;
    }

    if (tracker->amplitudeEstimate < kMinSignalAmplitude) {
        tracker->previousFilteredField = filteredField;
        if (tracker->stillTime > kStoppedTimeoutMin) {
            fieldFilter.reset();
            speedFilter.reset();
            resetTracker(tracker);
        }
        return 0.0f;
    }

    if (!tracker->hasEstimate) {
        float normalizedField = clamp(filteredField / tracker->amplitudeEstimate, -1.0f, 1.0f);
        tracker->quadratureEstimate = fieldSlope;
        tracker->phase = std::asin(normalizedField);
        tracker->previousPhase = tracker->phase;
        tracker->signalOmega = 0.0f;
        tracker->omegaSample1 = 0.0f;
        tracker->omegaSample2 = 0.0f;
        tracker->wheelSpeedEstimate = 0.0f;
        tracker->hasEstimate = true;
    }

    tracker->quadratureEstimate = lowPassStep(
        tracker->quadratureEstimate,
        fieldSlope,
        kDerivativeFilterTimeConstant,
        deltaTime
    );

    float normalizedField = clamp(filteredField / tracker->amplitudeEstimate, -1.0f, 1.0f);
    float principalPhase = std::asin(normalizedField);
    if (tracker->quadratureEstimate < 0.0f) {
        tracker->phase = kPi - principalPhase;
        if (tracker->phase > kPi) {
            tracker->phase -= 2.0f * kPi;
        }
    } else {
        tracker->phase = principalPhase;
    }

    float instantaneousSignalOmega = tracker->signalOmega;
    float cosineMagnitude = std::sqrt(fmaxf(0.0f, 1.0f - (normalizedField * normalizedField)));
    if (cosineMagnitude > 0.35f) {
        float derivativeSignalOmega = std::fabs(tracker->quadratureEstimate) /
                                      (tracker->amplitudeEstimate * cosineMagnitude);
        instantaneousSignalOmega = derivativeSignalOmega;
    }

    instantaneousSignalOmega = clamp(instantaneousSignalOmega, 0.0f, kMaxSignalOmega);

    float medianSignalOmega = median3(
        instantaneousSignalOmega,
        tracker->omegaSample1,
        tracker->omegaSample2
    );
    tracker->omegaSample2 = tracker->omegaSample1;
    tracker->omegaSample1 = instantaneousSignalOmega;

    tracker->signalOmega = lowPassStep(
        tracker->signalOmega,
        medianSignalOmega,
        kFrequencyFilterTimeConstant,
        deltaTime
    );
    tracker->previousPhase = tracker->phase;

    if (tracker->stillTime > kStoppedTimeoutMin) {
        fieldFilter.reset();
        speedFilter.reset();
        resetTracker(tracker);
        return 0.0f;
    }

    tracker->previousFilteredField = filteredField;

    float wheelAngularVelocity = tracker->signalOmega / kSignalCyclesPerWheelRevolution;
    float maxSpeedStep = kMaxWheelAcceleration * deltaTime;
    float limitedWheelSpeed = tracker->wheelSpeedEstimate;
    if (wheelAngularVelocity > limitedWheelSpeed + maxSpeedStep) {
        limitedWheelSpeed += maxSpeedStep;
    } else if (wheelAngularVelocity < limitedWheelSpeed - maxSpeedStep) {
        limitedWheelSpeed -= maxSpeedStep;
    } else {
        limitedWheelSpeed = wheelAngularVelocity;
    }

    tracker->wheelSpeedEstimate = lowPassStep(
        tracker->wheelSpeedEstimate,
        limitedWheelSpeed,
        kOutputFilterTimeConstant,
        deltaTime
    );
    speedFilter.add(tracker->wheelSpeedEstimate, deltaTime);

    float filteredAngularVelocity = speedFilter.get();
    if (filteredAngularVelocity < kZeroSpeedThreshold) {
        return 0.0f;
    }

    return filteredAngularVelocity;
}

void WheelMagnets::evaluate(VcuParameters *params, WheelMagnetsInput *input, WheelMagnetsOutput *output, float deltaTime) {
    output->ok = true;
    output->fault = WHEEL_MAGNETS_OK;

    if (!input->isSensorFlOk || !input->isSensorFrOk || !input->isSensorBlOk || !input->isSensorBrOk) {
        output->fault |= WHEEL_MAGNETS_MISSING;
        output->ok = false;
    }

    if (input->isSensorFrOk) {
        output->wheelSpeedFr = calcSpeed(&dispFr, input->wheelMagneticFieldFr, deltaTime, fieldFilterFr, speedFilterFr);
    } else {
        invalidateSensor(&dispFr, fieldFilterFr, speedFilterFr, &output->wheelSpeedFr);
    }

    if (input->isSensorFlOk) {
        output->wheelSpeedFl = calcSpeed(&dispFl, input->wheelMagneticFieldFl, deltaTime, fieldFilterFl, speedFilterFl);
    } else {
        invalidateSensor(&dispFl, fieldFilterFl, speedFilterFl, &output->wheelSpeedFl);
    }

    if (input->isSensorBrOk) {
        output->wheelSpeedBr = calcSpeed(&dispBr, input->wheelMagneticFieldBr, deltaTime, fieldFilterBr, speedFilterBr);
    } else {
        invalidateSensor(&dispBr, fieldFilterBr, speedFilterBr, &output->wheelSpeedBr);
    }

    if (input->isSensorBlOk) {
        output->wheelSpeedBl = calcSpeed(&dispBl, input->wheelMagneticFieldBl, deltaTime, fieldFilterBl, speedFilterBl);
    } else {
        invalidateSensor(&dispBl, fieldFilterBl, speedFilterBl, &output->wheelSpeedBl);
    }

}
