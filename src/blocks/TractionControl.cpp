#include "TractionControl.h"

/**
 * Reduce the driver's torque request when necessary to maintain traction.
 * This prevents the car from spinning out, giving the driver more confidence in acceleration.
 */

void TractionControl::setParameters(VcuParameters *params) {
    lowPassFeedback = LowPassFilter(params->tcsFeedbackLowPassFilterTimeConstant);
}

void TractionControl::evaluate(VcuParameters *params, TractionControlInput *input, TractionControlOutput *output,
                               float deltaTime) {
    if(!params->tcsEnabled || !input->wheelSpeedsOk) {
        output->regulatedTorqueRequest = input->unregulatedTorqueRequest;
        return;
    }

    float averageFrontVelocity = (input->wheelSpeedFl + input->wheelSpeedFr) / 2.0f;
    float excessVelocityL = input->wheelSpeedBl - averageFrontVelocity;
    float excessVelocityR = input->wheelSpeedBr - averageFrontVelocity;
    float excessVelocity = (excessVelocityL > excessVelocityR) ? excessVelocityL : excessVelocityR;
    float excessSlip = (excessVelocity / averageFrontVelocity) - 0.15f;

    if(excessSlip < 0 || excessVelocity < 0.01f) {
        excessSlip = 0;
    }
    excessVelocity = excessSlip * averageFrontVelocity;
    lowPassFeedback.add(excessVelocity, deltaTime);
    excessVelocity = lowPassFeedback.get();

    float negativeFeedback = 3.0f * excessVelocity;

    float maxTorque = 230.0f;
    float openLoopTorqueLimit = input->unregulatedTorqueRequest;
    if(input->unregulatedTorqueRequest > maxTorque) {
        openLoopTorqueLimit = maxTorque;
    }

    if(negativeFeedback > openLoopTorqueLimit) {
        negativeFeedback = openLoopTorqueLimit;
    }

    output->regulatedTorqueRequest = openLoopTorqueLimit - negativeFeedback;

}
