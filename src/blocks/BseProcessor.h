#ifndef VCU_CORE_BSEPROCESSOR_H
#define VCU_CORE_BSEPROCESSOR_H

#include "VcuParameters.h"

typedef struct BseProcessorInput {
    float bse1; // bse1 voltage (V)
    float bse2; // bse2 voltage (V)
} BseProcessorInput;

typedef struct BseProcessorOutput {
    float bse; // brake pressure (psi)
    bool ok;
} BseProcessorOutput;

class BseProcessor {
public:
    void setParameters(VcuParameters* params) {};
    void evaluate(VcuParameters* params, BseProcessorInput* input, BseProcessorOutput* output, float deltaTime);
private:
    Timer clock = Timer(0.1f);
    LowPassFilter bse1Filter = LowPassFilter(0.005);
    LowPassFilter bse2Filter = LowPassFilter(0.005);
};


#endif //VCU_CORE_BSEPROCESSOR_H
