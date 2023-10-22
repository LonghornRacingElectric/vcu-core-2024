#ifndef VCU_CORE_APPSPROCESSOR_H
#define VCU_CORE_APPSPROCESSOR_H

#include "VcuParameters.h"
#include "util/filters/Timer.h"
#include "util/filters/LowPassFilter.h"

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
    void reset();
    void evaluate(VcuParameters* params, AppsProcessorInput* input, AppsProcessorOutput* output, float deltaTime);
    void setParameters(VcuParameters* params);
private:
    Timer rangeClock = Timer(0.1f);
    Timer differenceClock = Timer(0.1f);
    LowPassFilter app1Filter = LowPassFilter(0);
    LowPassFilter app2Filter = LowPassFilter(0);
};


#endif //VCU_CORE_APPSPROCESSOR_H
