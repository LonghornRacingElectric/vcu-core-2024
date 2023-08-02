#ifndef VCU_CORE_2024_VCU_CORE_H
#define VCU_CORE_2024_VCU_CORE_H


#include "VcuParameters.h"
#include "blocks/AppsProcessor.h"

// https://www.figma.com/file/z98vFbTBytWElKBb5sTkwk/Lapsim-v2024-Architecture?type=whiteboard&node-id=0%3A1&t=05V7HnOWgZNBNB2B-1

typedef struct VcuInputs {
    float apps1;
    float apps2;
} VcuInputs;

typedef struct VcuOutputs {

} VcuOutputs;

class VcuModel {
private:
    AppsProcessor appsProcessor;
    BseProcessor bseProcessor;
    Stompp stompp;
    TorqueMapping torqueMapping;
    TractionControl tractionControl;
    SoftwareShutdown softwareShutdown;

    AppsProcessorInput appsProcessorInput;
    AppsProcessorOutput appsProcessorOutput;
public:
    void evaluate(VcuParameters* params, VcuInputs* input, VcuOutputs* output, float deltaTime);
};


#endif //VCU_CORE_2024_VCU_CORE_H
