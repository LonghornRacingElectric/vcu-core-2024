#include "AppsProcessor.h"

/**
 * Take APPS raw analog voltages and convert them to pedal travel percentages.
 * Make sure the two sensors are both within range and in agreement with each other.
 *
 * Read the rules!
 */

void AppsProcessor::evaluate(VcuParameters* params, AppsProcessorInput* input,
                             AppsProcessorOutput* output, float deltaTime) {


    //check if inputs are within working range first

    //checking whether app1 input is greater than max voltage params or less than min voltage params
    if(input->apps1 < params->apps1VoltageMin)
    {
        output->ok = false;
        output->apps = 0;
        return;
    } else if (input->apps1 > params->apps1VoltageMax)
    {
        output->ok = false;
        output->apps = 100;
        return;
    }
    //checking whether app2 input is greater than max voltage params or less than min voltage params
    if(input->apps2 < params->apps2VoltageMin)
    {
        output->ok = false;
        output->apps = 0;
        return;
    } else if (input->apps2 > params->apps2VoltageMax)
    {
        output->ok = false;
        output->apps = 100;
        return;
    }
    // since percentage will be (input - min) / (max - min)
    // finding max - min
    float diff = params->apps1VoltageMax - params->apps1VoltageMin;
    // finding input - min
    float app1Perc = input->apps1 - params->apps1VoltageMin;
    float app2Perc = input->apps2 - params->apps2VoltageMin;
    // calc percentage
    app1Perc = app1Perc / diff;
    app2Perc = app2Perc / diff;
    // find perc difference and see if it is within 10%
    diff = app1Perc - app2Perc;
    // turn value to positive if negative
    if(diff < 0) {diff = diff * -1;}
    // check if values comply
    if(diff > params->appsPlausibilityRange)
    {
        output->ok = false;
        output->apps = app1Perc;
        return;
    }
    else
    {
        output->ok = true;
        output->apps = app1Perc;
        return;
    }
    // TODO use parameters
    // TODO implement plausibility
    // TODO add low-pass filters

}
