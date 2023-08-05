#include "SoftShutdown.h"

/**
 * Software-level shutdown of the motor. Kind of like the shutdown circuit but for the code.
 * Happens if any critical sensor reading is out of bounds or the driver presses both pedals for example.
 */

void SoftShutdown::evaluate(VcuParameters *params, SoftShutdownInput *input, SoftShutdownOutput *output,
                            float deltaTime) {

    bool everythingOk = input->appsOk && input->bseOk && input->stomppOk;

    // TODO look into Cascadia documentation and see if enable/disable is relevant here
    output->enableInverter = everythingOk;
    output->inverterTorqueRequest = (everythingOk) ? input->torqueRequest : 0;

}
