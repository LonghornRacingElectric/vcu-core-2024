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
    float excessL = wheelVelocityBl - averageFrontVelocity;
    float excessR = wheelVelocityBr - averageFrontVelocity;
    float excess = (excessL > excessR) ? excessL : excessR;
    excess = excess / averageFrontVelocity;
    if(excess < 0) excess = 0;

    if(wheelVelocityBl < 1.0f || wheelVelocityBr < 1.0f) {
        excess = 0.0f;
    }

    float negativeFeedback = 5.0f * excess;
    lowPassFeedback.add(negativeFeedback, deltaTime);
    negativeFeedback = lowPassFeedback.get();

    if(negativeFeedback > input->unregulatedTorqueRequest) {
        negativeFeedback = input->unregulatedTorqueRequest;
    }

    output->regulatedTorqueRequest = input->unregulatedTorqueRequest - negativeFeedback;

}
