#include "TorqueMap.h"

/**
 * Map a pedal travel percentage to a torque request.
 */

void TorqueMap::evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime) {
    float torqueRequest = params->mapPedalToTorqueRequest(input->apps);
    float derate;

//    derate = params->mapDerateMotorTemp(input->motorTemp);
//    torqueRequest *= derate;
//
//    derate = params->mapDerateInverterTemp(input->inverterTemp);
//    torqueRequest *= derate;
//
//    derate = params->mapDerateBatteryTemp(input->batteryTemp);
//    torqueRequest *= derate;
//
//    derate = params->mapDerateBatterySoc(input->batterySoc);
//    torqueRequest *= derate;
//

    if(input->batteryCurrent < 1.0f || input->batteryCurrent > 1.0f) {
        openCircuitVoltageFilter.add(input->batteryVoltage, deltaTime);
    }
    float openCircuitVoltage = openCircuitVoltageFilter.get();
    float internalResistance = 0.650; // Ohms, estimated

    float currentLimit = 230.0f; // Amps
    float currentBasedPowerLimit = (openCircuitVoltage - (currentLimit * internalResistance)) * currentLimit;
    float powerLimit = params->mapPowerLimit;
    if(currentBasedPowerLimit < powerLimit) {
        powerLimit = currentBasedPowerLimit;
    }

    float motorAngularVelocity = input->motorRpm / 60.0f * 2.0f * 3.14159f; // rad/s
    float maxTorqueAtPowerLimit = powerLimit / motorAngularVelocity * 0.95f; // Nm
    if(torqueRequest > maxTorqueAtPowerLimit) {
        torqueRequest = maxTorqueAtPowerLimit;
    }

    float currentPower = input->batteryVoltage * input->batteryCurrent;
    if(input->batteryCurrent > 240.0f || currentPower > 85000.0f) {
        torqueRequest = 0;
    }

//    float powerError = 0;
//    if (currentPower > params->mapPowerLimit) {
//        // acceleration power limit
//        powerError = params->mapPowerLimit - currentPower;
//    } else if (currentPower < -params->mapPowerLimit) {
//        // regen power limit
//        powerError = -params->mapPowerLimit - currentPower;
//    }
//    float negativeFeedback = powerError * params->mapPowerLimitFeedbackP;
//    if ((torqueRequest > 0 && torqueRequest + negativeFeedback < 0)
//        || (torqueRequest < 0 && torqueRequest + negativeFeedback > 0)
//        || (torqueRequest == 0)) {
//        torqueRequest = 0;
//    } else {
//        torqueRequest = torqueRequest + negativeFeedback;
//    }

//    if(input->batteryCurrent > 200)
//    {
//        torqueRequest = 0;
//    }

    output->torqueRequest = torqueRequest;
}

void TorqueMap::setParameters(VcuParameters *params) {

}
