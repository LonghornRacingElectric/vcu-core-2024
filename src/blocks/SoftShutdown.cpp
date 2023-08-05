#include "SoftShutdown.h"

void SoftShutdown::evaluate(VcuParameters *params, SoftShutdownInput *input, SoftShutdownOutput *output,
                            float deltaTime) {

    bool everythingOk = input->appsOk && input->bseOk && input->stomppOk;

    // TODO look into Cascadia documentation and see if enable/disable is relevant here
    output->enableInverter = everythingOk;
    output->inverterTorqueRequest = (everythingOk) ? input->torqueRequest : 0;

}
