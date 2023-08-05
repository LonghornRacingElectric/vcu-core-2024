#include "TractionControl.h"

void TractionControl::evaluate(VcuParameters *params, TractionControlInput *input, TractionControlOutput *output,
                               float deltaTime) {


    // TODO implement
    // passthrough for now
    output->regulatedTorqueRequest = input->unregulatedTorqueRequest;

}
