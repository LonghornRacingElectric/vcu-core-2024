#include "Cooling.h"

void Cooling::setParameters(VcuParameters *params) {

}

void Cooling::evaluate(VcuParameters *params, CoolingInput *input, CoolingOutput *output, float deltaTime) {
    // this code outputs max cooling all the time
    // TODO make cooling outputs depend on temperature inputs
    output->pumpOutput = 1.0f;
    output->radiatorOutput = 1.0f;
    output->batteryFansOutput = 1.0f;
}
