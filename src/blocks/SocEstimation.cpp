#include "SocEstimation.h"

void SocEstimation::setParameters(VcuParameters *params) {
  if (params->socHvRemembered > 0.0f) {
    this->hvBatterySoc = params->socHvRemembered;
  }
  if (params->socLvRemembered > 0.0f) {
    this->lvBatterySoc = params->socLvRemembered;
  }
}

void SocEstimation::estimate(float voltage, float current, float &soc, CurveParameter &curve, float totalCharge,
                             float lowCurrent, float deltaTime) {
  bool isCurrentLow = (current < lowCurrent) && (current > -lowCurrent);
  bool isSocValid = soc > 0.0f;

  if (isCurrentLow || !isSocValid) {
    soc = curve(voltage);
  } else {
    soc -= current * deltaTime / totalCharge;
  }
}

void SocEstimation::evaluate(VcuParameters *params, SocEstimationInput *input, SocEstimationOutput *output,
                             float deltaTime) {

  estimate(input->hvBatteryVoltage, input->hvBatteryCurrent, this->hvBatterySoc, params->socHvVoltageCurve,
           params->socHvCharge, params->socHvLowCurrent, deltaTime);
  output->hvBatterySoc = hvBatterySoc;
  params->socHvRemembered = hvBatterySoc;

  estimate(input->lvBatteryVoltage, input->lvBatteryCurrent, this->lvBatterySoc, params->socLvVoltageCurve,
           params->socLvCharge, params->socLvLowCurrent, deltaTime);
  output->lvBatterySoc = lvBatterySoc;
  params->socLvRemembered = lvBatterySoc;

}
