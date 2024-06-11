#ifndef VCU_CORE_TORQUEMAP_H
#define VCU_CORE_TORQUEMAP_H


#include "VcuParameters.h"
#include "util/filters/LowPassFilter.h"


typedef struct TorqueMapInput {
    float apps; // pedal travel (%)
    float motorTemp; // (deg C)
    float wheelSpeedBl; // (rad/s)
    float wheelSpeedBr; // (rad/s)
    float inverterTemp; // (deg C)
    float batteryTemp; // (deg C)
    float batterySoc; // (%)
    float batteryVoltage; // (V)
    float batteryCurrent; // (A)
} TorqueMapInput;


typedef struct TorqueMapOutput {
    float torqueRequest; // torque (Nm)
} TorqueMapOutput;


class TorqueMap {
private:
    LowPassFilter powerNegativeFeedbackFilter = LowPassFilter(0);
public:
    void setParameters(VcuParameters* params);
    void evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime);
};


#endif //VCU_CORE_TORQUEMAP_H
