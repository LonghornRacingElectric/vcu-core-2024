#ifndef VCU_CORE_BSEPROCESSOR_H
#define VCU_CORE_BSEPROCESSOR_H

#include "VcuParameters.h"
#include "util/filters/Timer.h"
#include "util/filters/LowPassFilter.h"

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
    void reset();
    void setParameters(VcuParameters* params);
    void evaluate(VcuParameters* params, BseProcessorInput* input, BseProcessorOutput* output, float deltaTime);
private:
    Timer clock = Timer(0);
    LowPassFilter bse1Filter = LowPassFilter(0);
    LowPassFilter bse2Filter = LowPassFilter(0);
};


#endif //VCU_CORE_BSEPROCESSOR_H
