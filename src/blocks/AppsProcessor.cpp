
#include "AppsProcessor.h"


/**
 * Take APPS raw analog voltages and convert them to pedal travel percentages.
 * Make sure the two sensors are both within range and in agreement with each other.
 *
 * Read the rules!
 */

void AppsProcessor::evaluate(VcuParameters *params, AppsProcessorInput *input,
                             AppsProcessorOutput *output, float deltaTime) {


    bool apps1InRange = (input->apps1 >= params->apps1VoltageMin) && (input->apps1 <= params->apps1VoltageMax);
    bool apps2InRange = (input->apps2 >= params->apps2VoltageMin) && (input->apps2 <= params->apps2VoltageMax);

    if (!apps1InRange || !apps2InRange) {
        rangeClock.count(deltaTime);
        if (rangeClock.isFinished()) {
            output->ok = false;
            app1Filter.reset();
            app2Filter.reset();
            return;
        }
    } else {
        rangeClock.reset();
    }

    if(apps1InRange) app1Filter.add(input->apps1, deltaTime);
    if(apps2InRange) app2Filter.add(input->apps2, deltaTime);

    float app1Perc = app1Filter.get() - params->apps1VoltageMin;
    float app2Perc = app2Filter.get() - params->apps2VoltageMin;
    app1Perc = app1Perc / (params->apps1VoltageMax - params->apps1VoltageMin);
    app2Perc = app2Perc / (params->apps2VoltageMax - params->apps2VoltageMin);

    float diff = app1Perc - app2Perc;
    if (diff < 0) { diff = -diff; }

    if (diff > params->appsPlausibilityRange) {
        differenceClock.count(deltaTime);
        if (differenceClock.isFinished()) {
            output->ok = false;
            app1Filter.reset();
            app2Filter.reset();
            return;
        }
    } else {
        differenceClock.reset();
    }

    output->ok = true;

    float appsNoDeadzone = (app1Perc + app2Perc) / 2;
    float slope = 1.0f / (1.0f - (2.0f * params->appsDeadZonePct));

    if (appsNoDeadzone <= params->appsDeadZonePct) {
        output->apps = 0;
    } else if (appsNoDeadzone >= (1 - params->appsDeadZonePct)) {
        output->apps = 1;
    } else {
        output->apps = slope * (appsNoDeadzone - params->appsDeadZonePct);
    }
}

void AppsProcessor::reset() {
    app1Filter.reset();
    app2Filter.reset();
    differenceClock.reset();
    rangeClock.reset();
}

void AppsProcessor::setParameters(VcuParameters *params) {
    this->differenceClock = Timer(params->appsImplausibilityTime);
    this->rangeClock = Timer(params->appsImplausibilityTime);
    this->app1Filter = LowPassFilter(params->appsLowPassFilterTimeConstant);
    this->app2Filter = LowPassFilter(params->appsLowPassFilterTimeConstant);
}
