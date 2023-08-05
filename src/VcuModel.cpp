#include "VcuModel.h"

void VcuModel::evaluate(VcuParameters *params, VcuInputs *vcuInput, VcuOutputs *vcuOutput, float deltaTime) {

    appsProcessorInput = {
            vcuInput->apps1,
            vcuInput->apps2,
    };
    appsProcessor.evaluate(params, &appsProcessorInput, &appsProcessorOutput, deltaTime);

    bseProcessorInput = {
            vcuInput->bse1,
            vcuInput->bse2,
    };
    bseProcessor.evaluate(params, &bseProcessorInput, &bseProcessorOutput, deltaTime);

    stomppInput = {
            appsProcessorOutput.apps,
            bseProcessorOutput.bse,
    };
    stompp.evaluate(params, &stomppInput, &stomppOutput, deltaTime);

    torqueMapInput = {
            appsProcessorOutput.apps,
    };
    torqueMap.evaluate(params, &torqueMapInput, &torqueMapOutput, deltaTime);

    tractionControlInput = {
            torqueMapOutput.torqueRequest,
            vcuInput->wheelSpeedFl,
            vcuInput->wheelSpeedFr,
            vcuInput->wheelSpeedBl,
            vcuInput->wheelSpeedBr,
    };
    tractionControl.evaluate(params, &tractionControlInput, &tractionControlOutput, deltaTime);

    softShutdownInput = {
            appsProcessorOutput.ok,
            bseProcessorOutput.ok,
            stomppOutput.ok,
            tractionControlOutput.regulatedTorqueRequest,
    };
    softShutdown.evaluate(params, &softShutdownInput, &softShutdownOutput, deltaTime);

    *vcuOutput = {
        softShutdownOutput.enableInverter,
        softShutdownOutput.inverterTorqueRequest,
    };

}
