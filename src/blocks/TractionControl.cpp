#include "TractionControl.h"

/**
 * Reduce the driver's torque request when necessary to maintain traction.
 * This prevents the car from spinning out, giving the driver more confidence in acceleration.
 */

void TractionControl::setParameters(VcuParameters *params) {
    lowPassFl = LowPassFilter(params->tcsVelocityLowPassFilterTimeConstant);
    lowPassFr = LowPassFilter(params->tcsVelocityLowPassFilterTimeConstant);
    lowPassBl = LowPassFilter(params->tcsVelocityLowPassFilterTimeConstant);
    lowPassBr = LowPassFilter(params->tcsVelocityLowPassFilterTimeConstant);

    lowPassFeedback = LowPassFilter(params->tcsFeedbackLowPassFilterTimeConstant);
}

void TractionControl::evaluate(VcuParameters *params, TractionControlInput *input, TractionControlOutput *output,
                               float deltaTime) {


    // TODO replace this stupid implementation
    float wheelVelocityFl = differentiatorFl.get(input->wheelDisplacementFl, deltaTime);
    float wheelVelocityFr = differentiatorFr.get(input->wheelDisplacementFr, deltaTime);
    float wheelVelocityBl = differentiatorBl.get(input->wheelDisplacementBl, deltaTime);
    float wheelVelocityBr = differentiatorBr.get(input->wheelDisplacementBr, deltaTime);

    lowPassFl.add(wheelVelocityFl, deltaTime);
    wheelVelocityFl = lowPassFl.get();
    lowPassFr.add(wheelVelocityFr, deltaTime);
    wheelVelocityFr = lowPassFr.get();
    lowPassBl.add(wheelVelocityBl, deltaTime);
    wheelVelocityBl = lowPassBl.get();
    lowPassBr.add(wheelVelocityBr, deltaTime);
    wheelVelocityBr = lowPassBr.get();

    float averageFrontVelocity = (wheelVelocityFl + wheelVelocityFr) / 2.0f;
    float excessVelocityL = wheelVelocityBl - averageFrontVelocity;
    float excessVelocityR = wheelVelocityBr - averageFrontVelocity;
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
