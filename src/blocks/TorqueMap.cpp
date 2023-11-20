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

    float currentPower = input->batteryVoltage * input->batteryCurrent;
    if(currentPower > params->mapPowerLimit) { // acceleration power limit
        float error = currentPower - params->mapPowerLimit;
        torqueRequest = torqueRequest - (params->mapPowerLimitFeedbackP * error);
        if(torqueRequest < 0) {
            torqueRequest = 0;
        }
    } else if(currentPower < -params->mapPowerLimit) { // regen power limit
        float error = currentPower + params->mapPowerLimit;
        torqueRequest = torqueRequest - (params->mapPowerLimitFeedbackP * error);
        if(torqueRequest > 0) {
            torqueRequest = 0;
        }
    }

    output->torqueRequest = torqueRequest;
}
