#ifndef VCU_CORE_VCUCORE_H
#define VCU_CORE_VCUCORE_H

#include "VcuModel.h"


void reset();
void setParameters(VcuParameters newVcuParameters);
VcuOutput evaluate(VcuInput input, float deltaTime);


#endif //VCU_CORE_VCUCORE_H
