#ifndef VCU_CORE_TRACTIONCONTROL_H
#define VCU_CORE_TRACTIONCONTROL_H


#include "VcuParameters.h"

#include "util/filters/LowPassFilter.h"
#include "util/filters/Differentiator.h"


typedef struct TractionControlInput {
    float unregulatedTorqueRequest; // torque (Nm)

    float wheelSpeedFl; // (rad)
    float wheelSpeedFr;
    float wheelSpeedBl;
    float wheelSpeedBr;

    bool wheelSpeedsOk;
} TractionControlInput;


typedef struct TractionControlOutput {
    float regulatedTorqueRequest; // torque (Nm)
} TractionControlOutput;


class TractionControl {
private:
    Differentiator differentiatorFl;
    Differentiator differentiatorFr;
    Differentiator differentiatorBl;
    Differentiator differentiatorBr;

    LowPassFilter lowPassFeedback = LowPassFilter(0);
public:
    void setParameters(VcuParameters* params);
    void evaluate(VcuParameters *params, TractionControlInput *input, TractionControlOutput *output, float deltaTime);
};


#endif //VCU_CORE_TRACTIONCONTROL_H
