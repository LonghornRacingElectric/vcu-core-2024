#include "BseProcessor.h"

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

    // TODO implement

    output->ok = true;

}
