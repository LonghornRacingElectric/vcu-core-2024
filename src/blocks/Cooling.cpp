#include "Cooling.h"

void Cooling::setParameters(VcuParameters *params) {

}

void Cooling::evaluate(VcuParameters *params, CoolingInput *input, CoolingOutput *output, float deltaTime) {
    output->pumpOutput = 1.0f;
    output->radiatorOutput = params->coolingRadiatorFanCurve(input->motorTemp);
    output->batteryFansOutput = params->coolingBatteryFanCurve(input->batteryTemp);
}
