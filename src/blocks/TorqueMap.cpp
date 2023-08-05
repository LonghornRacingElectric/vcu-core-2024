#include "TorqueMap.h"

void TorqueMap::evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime) {

    // TODO implement this properly
    output->torqueRequest = input->apps * 230;

}
