#include "BseProcessor.h"
#include "util/MathUtils.h"

/**
 * Take BSE raw analog voltages and convert them to brake pressures.
 * Make sure the two sensors are both within range and in agreement with each other.
 *
 * Technically we only need one functioning BSE for rules compliance so you
 * can discard one sensor's value if it's out of range and just use the other one.
 *
 * Read the rules!
 */



void BseProcessor::evaluate(VcuParameters *params, BseProcessorInput *input,
                            BseProcessorOutput *output, float deltaTime) {

    /*
     * Rules only requires one BSE, so this code will still allow the car to run in the scenario
     * that only one BSE of our two BSEs is working.
     */

    bool bse1Plausible = (input->bse1 >= params->bseVoltageMin) && (input->bse1 <= params->bseVoltageMax);
    bool bse2Plausible = (input->bse2 >= params->bseVoltageMin) && (input->bse2 <= params->bseVoltageMax);

    float bse = 0;
    float div = 0;

    if (bse1Plausible) {
        bse1Filter.add(input->bse1, deltaTime);
        float bse1Pressure = map(bse1Filter.get(), params->bseVoltageMin, params->bseVoltageMax,
                                 params->bsePressureMin, params->bsePressureMax);
        bse += bse1Pressure;
        div += 1.0f;
    } else {
        bse1Filter.reset();
    }

    if (bse2Plausible) {
        bse2Filter.add(input->bse2, deltaTime);
        float bse2Pressure = map(bse2Filter.get(), params->bseVoltageMin, params->bseVoltageMax,
                                 params->bsePressureMin, params->bsePressureMax);
        bse += bse2Pressure;
        div += 1.0f;
    } else {
        bse2Filter.reset();
    }

    if(div == 0) {
        clock.count(deltaTime);
        if(clock.isFinished()) {
            output->ok = false;
        } else {
            output->ok = true; // maintain last value for up to 100ms
        }
        return;
    } else {
        clock.reset();
    }

    output->bse = bse / div;
    output->ok = true;
}

void BseProcessor::setParameters(VcuParameters *params) {
    clock = Timer(params->bseImplausibilityTime);
    bse1Filter = LowPassFilter(params->bseLowPassFilterTimeConstant);
    bse2Filter = LowPassFilter(params->bseLowPassFilterTimeConstant);
}

void BseProcessor::reset() {
    clock.reset();
    bse1Filter.reset();
    bse2Filter.reset();
}
