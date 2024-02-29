#include "WheelMagnets.h"

void WheelMagnets::setParameters(VcuParameters *params) {
  speedFilterFl = LowPassFilter(params->wheelMagnetLpfTimeConstant);
  speedFilterFr = LowPassFilter(params->wheelMagnetLpfTimeConstant);
  speedFilterBl = LowPassFilter(params->wheelMagnetLpfTimeConstant);
  speedFilterBr = LowPassFilter(params->wheelMagnetLpfTimeConstant);
}

void WheelMagnets::evaluate(VcuParameters *params, WheelMagnetsInput *input, WheelMagnetsOutput *output, float deltaTime) {
  output->ok = true;
  output->fault = WHEEL_MAGNETS_OK;
  output->wheelSpeedFr = 0;
  output->wheelSpeedFl = 0;
  output->wheelSpeedBr = 0;
  output->wheelSpeedBl = 0;

  if(!input->areSensorsOk) {
    output->fault |= WHEEL_MAGNETS_MISSING;
    output->ok = false;
    return;
  }

  // TODO migrate wheel speed code from unsprung board here
}
