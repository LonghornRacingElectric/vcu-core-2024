#ifndef VCU_CORE_WHEELMAGNETS_H
#define VCU_CORE_WHEELMAGNETS_H

#include <stdint.h>

#include "VcuParameters.h"
#include "util/filters/LowPassFilter.h"

#define WHEEL_MAGNETS_OK 0x00
#define WHEEL_MAGNETS_MISSING 0x01

typedef struct WheelMagnetsInput {
  float wheelMagneticFieldFl; // magnetic flux density (mT)
  float wheelMagneticFieldFr;
  float wheelMagneticFieldBl;
  float wheelMagneticFieldBr;
  bool isSensorFlOk;
  bool isSensorFrOk;
  bool isSensorBlOk;
  bool isSensorBrOk;
} WheelMagnetsInput;


typedef struct WheelMagnetsOutput {
  float wheelSpeedFl; // rotational velocity (rad/s)
  float wheelSpeedFr;
  float wheelSpeedBl;
  float wheelSpeedBr;
  uint32_t fault;
  bool ok;
} WheelMagnetsOutput;

typedef struct WheelTracker{
    bool hasEstimate;
    bool hasPreviousSample;
    float phase;
    float previousPhase;
    float signalOmega;
    float omegaSample1;
    float omegaSample2;
    float wheelSpeedEstimate;
    float previousFilteredField;
    float amplitudeEstimate;
    float baselineEstimate;
    float quadratureEstimate;
    float activityEstimate;
    float directionEstimate;
    float crossingPeriod;
    float timeSinceCrossing;
    uint8_t consistentCrossings;
    int8_t signalBand;
    bool risingEdgeArmed;
    float stillTime;
}WheelTracker;


class WheelMagnets {
private:
  LowPassFilter fieldFilterFl = LowPassFilter(0.0f);
  LowPassFilter fieldFilterFr = LowPassFilter(0.0f);
  LowPassFilter fieldFilterBl = LowPassFilter(0.0f);
  LowPassFilter fieldFilterBr = LowPassFilter(0.0f);
  LowPassFilter speedFilterFl = LowPassFilter(0.0f);
  LowPassFilter speedFilterFr = LowPassFilter(0.0f);
  LowPassFilter speedFilterBl = LowPassFilter(0.0f);
  LowPassFilter speedFilterBr = LowPassFilter(0.0f);
public:
  void setParameters(VcuParameters *params);

  void evaluate(VcuParameters *params, WheelMagnetsInput *input, WheelMagnetsOutput *output, float deltaTime);
};


#endif //VCU_CORE_WHEELMAGNETS_H
