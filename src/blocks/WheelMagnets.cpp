#include "WheelMagnets.h"

float thresh = 10.0f;

WheelTracker dispFr{false, 0, 0, 0};
WheelTracker dispFl{false, 0, 0, 0};
WheelTracker dispBr{false, 0, 0, 0};
WheelTracker dispBl{false, 0, 0, 0};

void WheelMagnets::setParameters(VcuParameters *params) {
    speedFilterFl = LowPassFilter(params->wheelMagnetLpfTimeConstant);
    speedFilterFr = LowPassFilter(params->wheelMagnetLpfTimeConstant);
    speedFilterBl = LowPassFilter(params->wheelMagnetLpfTimeConstant);
    speedFilterBr = LowPassFilter(params->wheelMagnetLpfTimeConstant);
}


float calcSpeed(WheelTracker *tracker, float field, Differentiator& differentiator,
                float deltaTime, LowPassFilter& filter) {
    if (!tracker->isHigh && field > thresh) {
        tracker->isHigh = true;
        tracker->displacement += (M_PI) / 3.0f;
    } else if (tracker->isHigh && field < -1 * thresh) {
        tracker->isHigh = false;
        tracker->displacement += (M_PI) / 3.0f;
    }
    tracker->lastTime += deltaTime;
    float dydx = differentiator.get(tracker->displacement, tracker->lastTime);
    if(dydx != 0){
        filter.add(dydx, tracker->lastTime);
        tracker->lastTime = tracker->lastTime;
        tracker->curTime = 0;
    }
    return filter.get();
}

void WheelMagnets::evaluate(VcuParameters *params, WheelMagnetsInput *input, WheelMagnetsOutput *output, float deltaTime) {
    output->ok = true;
    output->fault = WHEEL_MAGNETS_OK;
//  output->wheelSpeedFr = 0;
//  output->wheelSpeedFl = 0;
//  output->wheelSpeedBr = 0;
//  output->wheelSpeedBl = 0;


    if (!input->areSensorsOk) {
        output->fault |= WHEEL_MAGNETS_MISSING;
        output->ok = false;
        return;
    }

    output->wheelSpeedFr = calcSpeed(&dispFr, input->wheelMagneticFieldFr, differentiatorFr, deltaTime, speedFilterFr);
    output->wheelSpeedFl = calcSpeed(&dispFl, input->wheelMagneticFieldFl, differentiatorFl, deltaTime, speedFilterFl);
    output->wheelSpeedBr = calcSpeed(&dispBr, input->wheelMagneticFieldBr, differentiatorBr, deltaTime, speedFilterBr);
    output->wheelSpeedBl = calcSpeed(&dispBl, input->wheelMagneticFieldBl, differentiatorBl, deltaTime, speedFilterBl);

}

