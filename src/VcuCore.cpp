#include "VcuCore.h"

static VcuModel vcuModel;
static VcuParameters vcuParameters;

void reset() {
    vcuModel = {};
}

void setParameters(VcuParameters newVcuParameters) {
    vcuParameters = newVcuParameters;
}

VcuOutput evaluate(VcuInput input, float deltaTime) {
    VcuOutput output;
    vcuModel.evaluate(&vcuParameters, &input, &output, deltaTime);
    return output;
}
