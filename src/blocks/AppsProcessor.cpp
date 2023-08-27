#include "AppsProcessor.h"
#include "util/filters/Timer.h"

/**
 * Take APPS raw analog voltages and convert them to pedal travel percentages.
 * Make sure the two sensors are both within range and in agreement with each other.
 *
 * Read the rules!
 */
Timer clock(0.1);
void AppsProcessor::evaluate(VcuParameters* params, AppsProcessorInput* input,
                             AppsProcessorOutput* output, float deltaTime) {


    /**
     *
     *
     *
     */
    //check if inputs are within working range first

    //checking whether app1 input is greater than max voltage params or less than min voltage params
    if(input->apps1 < params->apps1VoltageMin)
    {
        output->ok = false;
        return;
    } else if (input->apps1 > params->apps1VoltageMax)
    {
        output->ok = false;
        return;
    }
    //checking whether app2 input is greater than max voltage params or less than min voltage params
    if(input->apps2 < params->apps2VoltageMin)
    {
        output->ok = false;
        return;
    } else if (input->apps2 > params->apps2VoltageMax)
    {
        output->ok = false;
        return;
    }
    // check if we need evaluation or not
    
    // since percentage will be (input - min) / (max - min)
    // finding max - min
    float diff = params->apps1VoltageMax - params->apps1VoltageMin;
    // finding input - min
    float app1Perc = input->apps1 - params->apps1VoltageMin;
    float app2Perc = input->apps2 - params->apps2VoltageMin;
    // calc percentage
    app1Perc = app1Perc / diff;
    diff = params->apps2VoltageMax - params->apps2VoltageMin;
    app2Perc = app2Perc / diff;
    // find perc difference and see if it is within 10%
    diff = app1Perc - app2Perc;
    // turn value to positive if negative
    if(diff < 0) {diff = -diff;}
    // check if values comply
    if(diff > params->appsPlausibilityRange)
    {
        // check if clock is finished
        clock.count(deltaTime);
        if(clock.isFinished())
        {
            output->ok = false;
            clock.reset();
            return;
        }


    }
    else {
        output->ok = true;
        float preconv = (app1Perc + app2Perc) / 2; // TODO average
        // dead zone min
        if (preconv <= 0.05)
        {
            output->apps = 0;
        }
        // dead zone max
        else if (preconv >= 0.95)
        {
            output->apps = 1;
        }
        // function to map from 0.05 -> 0.95
        else
        {
            output->apps = 1.111111*(preconv - 0.05);
        }
        return;
    }
    // TODO add low-pass filters

}
