#include "AppsProcessor.h"

/**
 * Take APPS raw analog voltages and convert them to pedal travel percentages.
 * Make sure the two sensors are both within range and in agreement with each other.
 *
 * Read the rules!
 */

void AppsProcessor::evaluate(VcuParameters* params, AppsProcessorInput* input,
                             AppsProcessorOutput* output, float deltaTime) {

    output->apps = (input->apps1 + input->apps2) / 2.0f; // do not use lmao
    output->ok = false;

    // TODO use parameters
    // TODO implement plausibility
    // TODO add low-pass filters

}
