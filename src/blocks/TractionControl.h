#ifndef VCU_CORE_TRACTIONCONTROL_H
#define VCU_CORE_TRACTIONCONTROL_H


#include "VcuParameters.h"


typedef struct TractionControlInput {
    float unregulatedTorqueRequest; // torque (Nm)

    float wheelSpeedFl; // (rad/s)
    float wheelSpeedFr;
    float wheelSpeedBl;
    float wheelSpeedBr;
} TractionControlInput;


typedef struct TractionControlOutput {
    float regulatedTorqueRequest; // torque (Nm)
} TractionControlOutput;


class TractionControl {
public:
    void setParameters(VcuParameters* params) {};
    void evaluate(VcuParameters *params, TractionControlInput *input, TractionControlOutput *output, float deltaTime);
};


#endif //VCU_CORE_TRACTIONCONTROL_H
