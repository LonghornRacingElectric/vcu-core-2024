#include "AppsProcessor.h"

void AppsProcessor::evaluate(VcuParameters* params, AppsProcessorInput* input,
                                            AppsProcessorOutput* output, float deltaTime) {

    output->apps = (input->apps1 + input->apps2) / 2.0f; // do not use lmao
    output->plausible = false;
    // TODO implement plausibility
    // TODO low-pass filter

}
