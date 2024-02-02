#ifndef VCU_CORE_INDICATORS_H
#define VCU_CORE_INDICATORS_H


#include "VcuParameters.h"


typedef struct IndicatorsInput {
    float bse;
} IndicatorsInput;


typedef struct IndicatorsOutput {
    bool brakeLight;
} IndicatorsOutput;


class Indicators {
public:
    void setParameters(VcuParameters* params);
    void evaluate(VcuParameters *params, IndicatorsInput *input, IndicatorsOutput *output, float deltaTime);
};


#endif //VCU_CORE_INDICATORS_H
