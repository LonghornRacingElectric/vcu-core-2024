#include "VcuModel.h"

void VcuModel::evaluate(VcuParameters* params, VcuInputs* input, VcuOutputs* output, float deltaTime) {
    appsProcessorInput = { input->apps1, input->apps2 };
    appsProcessorOutput = {};
    appsProcessor.evaluate(params, &appsProcessorInput, &appsProcessorOutput, deltaTime);

    bseProcessorInput = {};
    bseProcessorOutput = {};
    bseProcessor.evaluate(params, &bseProcessorInput, &bseProcessorOutput, deltaTime);

    stomppInput = {};
    stomppOutput = {};
    stompp.evaluate(params, &stomppInput, &stomppOutput, deltaTime);

    torqueMappingInput = {};
    torqueMappingOutput = {};
    torqueMapping.evaluate(params, &torqueMappingInput, &torqueMappingOutput, deltaTime);

    tractionControlInput = {};
    tractionControlOutput = {};
    tractionControl.evaluate(params, &tractionControlInput, &tractionControlOutput, deltaTime);

    softwareShutdownInput = {};
    softwareShutdownOutput = {};
    softwareShutdown.evaluate(params, &softwareShutdownInput, &softwareShutdownOutput, deltaTime);

    output = {

    };
}
