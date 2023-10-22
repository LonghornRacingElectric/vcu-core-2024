
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
    output->ok = true;
    //checking whether app1 input is greater than max voltage params or less than min voltage params
    if(input->apps1 < params->apps1VoltageMin)
    {
        output->ok = false;

    } else if (input->apps1 > params->apps1VoltageMax)
    {
        output->ok = false;

    }
    //checking whether app2 input is greater than max voltage params or less than min voltage params
    if(input->apps2 < params->apps2VoltageMin)
    {
        output->ok = false;

    } else if (input->apps2 > params->apps2VoltageMax)
    {
        output->ok = false;

    }
    if(!output->ok)
    {
        app1Filter.reset();
        app2Filter.reset();
        return;
    }

    // add low pass filter
    app1Filter.add(input->apps1, deltaTime);
    app2Filter.add(input->apps2, deltaTime);
    // check if we need evaluation or not

    // since percentage will be (input - min) / (max - min)
    // finding max - min
    // finding input - min
    float app1Perc = app1Filter.get() - params->apps1VoltageMin;
    float app2Perc = app2Filter.get() - params->apps2VoltageMin;
    // calc percentage
    app1Perc = app1Perc / (params->apps1VoltageMax - params->apps1VoltageMin);
    app2Perc = app2Perc / (params->apps2VoltageMax - params->apps2VoltageMin);
    // find perc difference and see if it is within 10%
    float diff = app1Perc - app2Perc;
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
            app1Filter.reset();
            app2Filter.reset();
            return;
        }
        else
        {
            output->ok = true;
            return;
        }


    }
    else {
        output->ok = true;
        float preconv = (app1Perc + app2Perc) / 2;
        // calculate slope
        float slope = 1.0f/(1.0f - (2.0f*params->appsDeadZonePct));
        // dead zone min
        if (preconv <= params->appsDeadZonePct)
        {
            output->apps = 0;
        }
        // dead zone max
        else if (preconv >= (1 - params->appsDeadZonePct))
        {
            output->apps = 1;
        }
        // function to map from 0.05 -> 0.95
        else
        {
            output->apps = slope*(preconv - params->appsDeadZonePct);
            clock.reset();
        }
        return;
    }
}

void AppsProcessor::reset()
{
    app1Filter.reset();
    app2Filter.reset();
    clock.reset();
}

void AppsProcessor::setParameters(VcuParameters *params) {
    this->clock = Timer(params->appsImplausibilityTime);
    this->app1Filter = LowPassFilter(params->appsLowPassFilterTimeConstant);
    this->app2Filter = LowPassFilter(params->appsLowPassFilterTimeConstant);
}
