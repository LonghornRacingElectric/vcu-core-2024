#ifndef VCU_CORE_APPSPROCESSOR_H
#define VCU_CORE_APPSPROCESSOR_H

#include "VcuParameters.h"

typedef struct AppsProcessorInput {
    float apps1;
    float apps2;
} AppsProcessorInput;

typedef struct AppsProcessorOutput {
    float apps;
    bool plausible;
} AppsProcessorOutput;

class AppsProcessor {
public:
    void evaluate(VcuParameters* params, AppsProcessorInput* input, AppsProcessorOutput* output, float deltaTime);
};


#endif //VCU_CORE_APPSPROCESSOR_H
