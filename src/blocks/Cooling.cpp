#include "Cooling.h"

void Cooling::setParameters(VcuParameters *params) {

}

void Cooling::evaluate(VcuParameters *params, CoolingInput *input, CoolingOutput *output, float deltaTime) {
    // this code outputs max cooling all the time
    output->pumpOutput = 1.0f;
    output->radiatorOutput = params->coolingMotor(input->motorTemp);
    output->batteryFansOutput = params->coolingMotor(input->batteryTemp);
}
