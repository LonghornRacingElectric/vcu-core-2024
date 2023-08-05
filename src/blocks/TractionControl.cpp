#include "TractionControl.h"

/**
 * Reduce the driver's torque request when necessary to maintain traction.
 * This prevents the car from spinning out, giving the driver more confidence in acceleration.
 */

void TractionControl::evaluate(VcuParameters *params, TractionControlInput *input, TractionControlOutput *output,
                               float deltaTime) {


    // TODO implement
    // passthrough for now
    output->regulatedTorqueRequest = input->unregulatedTorqueRequest;

}
