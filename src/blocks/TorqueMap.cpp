#include "TorqueMap.h"

/**
 * Map a pedal travel percentage to a torque request.
 */

void TorqueMap::evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime) {
    float torqueRequest = params->mapPedalToTorqueRequest(input->apps);
    float derate;

//    float wheelSpeedAvg = (input->wheelSpeedBl + input->wheelSpeedBr) / 2.0f;
//    if(wheelSpeedAvg < params->mapLowSpeedDerateEnd) {
//      derate = wheelSpeedAvg / params->mapLowSpeedDerateEnd;
//      if(derate < params->mapLowSpeedDerateMin) derate = params->mapLowSpeedDerateMin;
//      torqueRequest *= derate;
//    }

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
//    float currentPower = input->batteryVoltage * input->batteryCurrent;
//    float powerError = 0;
//    if (currentPower > params->mapPowerLimit) {
//        // acceleration power limit
//        powerError = params->mapPowerLimit - currentPower;
//    } else if (currentPower < -params->mapPowerLimit) {
//        // regen power limit
//        powerError = -params->mapPowerLimit - currentPower;
//    }
//    powerNegativeFeedbackFilter.add(powerError, deltaTime);
//    float negativeFeedback = powerNegativeFeedbackFilter.get() * params->mapPowerLimitFeedbackP;
//    if ((torqueRequest > 0 && torqueRequest + negativeFeedback < 0)
//        || (torqueRequest < 0 && torqueRequest + negativeFeedback > 0)
//        || (torqueRequest == 0)) {
//        torqueRequest = 0;
//    } else {
//        torqueRequest = torqueRequest + negativeFeedback;
//    }

    output->torqueRequest = torqueRequest;
}

void TorqueMap::setParameters(VcuParameters *params) {
    this->powerNegativeFeedbackFilter = LowPassFilter(params->mapPowerLimitFeedbackTimeConstant);
}
