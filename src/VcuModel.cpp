#include "VcuModel.h"

void VcuModel::evaluate(VcuParameters *params, VcuInputs *vcuInput, VcuOutputs *vcuOutput, float deltaTime) {

    appsProcessorInput = {vcuInput->apps1, vcuInput->apps2};
    appsProcessorOutput = {};
    appsProcessor.evaluate(params, &appsProcessorInput, &appsProcessorOutput, deltaTime);

    bseProcessorInput = {vcuInput->bse1, vcuInput->bse2};
    bseProcessorOutput = {};
    bseProcessor.evaluate(params, &bseProcessorInput, &bseProcessorOutput, deltaTime);

    stomppInput = {appsProcessorOutput.apps, bseProcessorOutput.bse};
    stomppOutput = {};
    stompp.evaluate(params, &stomppInput, &stomppOutput, deltaTime);

    torqueMapInput = {appsProcessorOutput.apps};
    torqueMapOutput = {};
    torqueMap.evaluate(params, &torqueMapInput, &torqueMapOutput, deltaTime);

    tractionControlInput = {torqueMapOutput.torqueRequest};
    tractionControlOutput = {};
    tractionControl.evaluate(params, &tractionControlInput, &tractionControlOutput, deltaTime);

    softShutdownInput = {};
    softShutdownOutput = {};
    softShutdown.evaluate(params, &softShutdownInput, &softShutdownOutput, deltaTime);

    *vcuOutput = {
        softShutdownOutput.enableInverter,
        softShutdownOutput.inverterTorqueRequest
    };

}
