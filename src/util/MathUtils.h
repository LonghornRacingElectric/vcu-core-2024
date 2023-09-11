#include <math.h>

#ifndef VCU_CORE_MATHUTILS_H
#define VCU_CORE_MATHUTILS_H

static constexpr float map(const float value, const float fromLow, const float fromHigh, 
                            const float toLow, const float toHigh) {
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

static constexpr float percent(const float a, const float b) {
    return fabs(b-a) / ((b+a)/2);
}

#endif