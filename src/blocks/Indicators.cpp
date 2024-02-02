#include "Indicators.h"

void Indicators::setParameters(VcuParameters *params) {

}

void Indicators::evaluate(VcuParameters *params, IndicatorsInput *input, IndicatorsOutput *output, float deltaTime) {
    output->brakeLight = (input->bse >= params->stomppMechanicalBrakesThreshold);
}
