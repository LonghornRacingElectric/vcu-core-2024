#ifndef VCU_CORE_TRACTIONCONTROL_H
#define VCU_CORE_TRACTIONCONTROL_H


#include "VcuParameters.h"

#include "util/filters/LowPassFilter.h"
#include "util/filters/Differentiator.h"


typedef struct TractionControlInput {
    float unregulatedTorqueRequest; // torque (Nm)

    float wheelDisplacementFl; // (rad)
    float wheelDisplacementFr;
    float wheelDisplacementBl;
    float wheelDisplacementBr;
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

    LowPassFilter lowPassFl = LowPassFilter(0);
    LowPassFilter lowPassFr = LowPassFilter(0);
    LowPassFilter lowPassBl = LowPassFilter(0);
    LowPassFilter lowPassBr = LowPassFilter(0);

    LowPassFilter lowPassFeedback = LowPassFilter(0);
public:
    void setParameters(VcuParameters* params);
    void evaluate(VcuParameters *params, TractionControlInput *input, TractionControlOutput *output, float deltaTime);
};


#endif //VCU_CORE_TRACTIONCONTROL_H
