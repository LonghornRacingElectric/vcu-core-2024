#ifndef VCU_CORE_APPSPROCESSOR_H
#define VCU_CORE_APPSPROCESSOR_H

#include "parameters/VcuParameters.h"

typedef struct AppsProcessorInput {
    float apps1; // apps1 voltage (V)
    float apps2; // apps2 voltage (V)
} AppsProcessorInput;

typedef struct AppsProcessorOutput {
    float apps; // pedal travel (%)
    bool ok;
} AppsProcessorOutput;

class AppsProcessor {
public:
    void evaluate(VcuParameters* params, AppsProcessorInput* input, AppsProcessorOutput* output, float deltaTime);
};


#endif //VCU_CORE_APPSPROCESSOR_H
