#ifndef VCU_CORE_TORQUEMAP_H
#define VCU_CORE_TORQUEMAP_H


#include "VcuParameters.h"


typedef struct TorqueMapInput {
    float apps; // pedal travel (%)
    float motorTemp; // (deg C)
    float inverterTemp; // (deg C)
    float batteryTemp; // (deg C)
    float batterySoc; // (%)
} TorqueMapInput;


typedef struct TorqueMapOutput {
    float torqueRequest; // torque (Nm)
} TorqueMapOutput;


class TorqueMap {
public:
    void setParameters(VcuParameters* params) {};
    void evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime);
};


#endif //VCU_CORE_TORQUEMAP_H
