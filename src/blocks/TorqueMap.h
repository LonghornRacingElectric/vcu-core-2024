#ifndef VCU_CORE_TORQUEMAP_H
#define VCU_CORE_TORQUEMAP_H


#include "VcuParameters.h"


typedef struct TorqueMapInput {
    float apps; // pedal travel (%)
} TorqueMapInput;


typedef struct TorqueMapOutput {
    float torqueRequest; // torque (Nm)
} TorqueMapOutput;


class TorqueMap {
public:
    void evaluate(VcuParameters *params, TorqueMapInput *input, TorqueMapOutput *output, float deltaTime);
};


#endif //VCU_CORE_TORQUEMAP_H
