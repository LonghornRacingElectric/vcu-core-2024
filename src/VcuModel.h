#ifndef VCU_CORE_2024_VCU_CORE_H
#define VCU_CORE_2024_VCU_CORE_H


#include "VcuParameters.h"
#include "blocks/AppsProcessor.h"
#include "blocks/BseProcessor.h"
#include "blocks/Stompp.h"
#include "blocks/TorqueMap.h"
#include "blocks/TractionControl.h"
#include "blocks/SoftShutdown.h"

// https://www.figma.com/file/z98vFbTBytWElKBb5sTkwk/Lapsim-v2024-Architecture?type=whiteboard&node-id=0%3A1&t=05V7HnOWgZNBNB2B-1


typedef struct VcuInput {
    float apps1; // apps1 voltage (V)
    float apps2; // apps2 voltage (V)

    float bse1; // bse1 voltage (V)
    float bse2; // bse2 voltage (V)

    float wheelSpeedFl; // wheel speed (rad/s)
    float wheelSpeedFr;
    float wheelSpeedBl;
    float wheelSpeedBr;
} VcuInput;


typedef struct VcuOutput {
    bool enableInverter;
    float inverterTorqueRequest; // torque (Nm)
} VcuOutput;


class VcuModel {
private:
    AppsProcessor appsProcessor;
    AppsProcessorInput appsProcessorInput;
    AppsProcessorOutput appsProcessorOutput;

    BseProcessor bseProcessor;
    BseProcessorInput bseProcessorInput;
    BseProcessorOutput bseProcessorOutput;

    Stompp stompp;
    StomppInput stomppInput;
    StomppOutput stomppOutput;

    TorqueMap torqueMap;
    TorqueMapInput torqueMapInput;
    TorqueMapOutput torqueMapOutput;

    TractionControl tractionControl;
    TractionControlInput tractionControlInput;
    TractionControlOutput tractionControlOutput;

    SoftShutdown softShutdown;
    SoftShutdownInput softShutdownInput;
    SoftShutdownOutput softShutdownOutput;

public:
    void evaluate(VcuParameters* params, VcuInput* vcuInput, VcuOutput* vcuOutput, float deltaTime);
};


#endif //VCU_CORE_2024_VCU_CORE_H
