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

    output->ok = (input->bse1 >= bseVoltageMin && input->bse2 >= bseVoltageMin 
            && input->bse1 <= bseVoltageMax && input->bse2 <= bseVoltageMax);
    if (!output->ok) {
        bse1Filter.reset();
        bse2Filter.reset();
        return;
    }
    
    bse1Filter.add(input->bse1, deltaTime);
    bse2Filter.add(input->bse2, deltaTime);

    float bse1Pressure = map(bse1Filter.get(), bseVoltageMin, bseVoltageMax, bsePressureMin, bsePressureMax);
    float bse2Pressure = map(bse2Filter.get(), bseVoltageMin, bseVoltageMax, bsePressureMin, bsePressureMax);
    // float perc = percent(bse1Pressure, bse2Pressure);

    output->bse = (bse1Pressure + bse2Pressure) / 2;
}
