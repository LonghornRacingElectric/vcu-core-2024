#ifndef VCU_CORE_PRNDL_H
#define VCU_CORE_PRNDL_H


#include "VcuParameters.h"


typedef struct PrndlInput {
    bool driveSwitch; // true = drive
    float brakePressure;
    bool inverterReady;
} PrndlInput;


typedef struct PrndlOutput {
    bool state; // true = drive
    bool buzzer; // true = buzz
} PrndlOutput;


class Prndl {
public:
    void setParameters(VcuParameters* params) {};
    void evaluate(VcuParameters *params, PrndlInput *input, PrndlOutput *output, float deltaTime);
};


#endif //VCU_CORE_PRNDL_H
