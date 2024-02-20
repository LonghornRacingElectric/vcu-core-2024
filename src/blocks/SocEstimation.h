#ifndef VCU_CORE_SOCESTIMATION_H
#define VCU_CORE_SOCESTIMATION_H


#include "VcuParameters.h"


typedef struct SocEstimationInput {
  float hvBatteryVoltage;
  float hvBatteryCurrent;
  float lvBatteryVoltage;
  float lvBatteryCurrent;
} SocEstimationInput;


typedef struct SocEstimationOutput {
  float hvBatterySoc;
  float lvBatterySoc;
} SocEstimationOutput;


class SocEstimation {
private:
  float hvBatterySoc = -1.0f;
  float lvBatterySoc = -1.0f;

  static void estimate(float voltage, float current, float &soc, CurveParameter &curve, float totalCharge, float lowCurrent,
                float deltaTime);

public:
  void setParameters(VcuParameters *params);

  void evaluate(VcuParameters *params, SocEstimationInput *input, SocEstimationOutput *output, float deltaTime);
};


#endif //VCU_CORE_SOCESTIMATION_H
