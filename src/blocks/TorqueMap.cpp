#include "TorqueMap.h"

/**
 * Map a pedal travel percentage to a torque request.
 */

void TorqueMap::evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime) {
    float torqueRequest = params->mapPedalToTorqueRequest(input->apps);
    float derate;

    derate = params->mapDerateMotorTemp(input->motorTemp);
    torqueRequest *= derate;

    derate = params->mapDerateInverterTemp(input->inverterTemp);
    torqueRequest *= derate;

    derate = params->mapDerateBatteryTemp(input->batteryTemp);
    torqueRequest *= derate;

    derate = params->mapDerateBatterySoc(input->batterySoc);
    torqueRequest *= derate;

    output->torqueRequest = torqueRequest;
}
