#include "CurveParameter.h"
#include <cmath>
#include <cstring>

CurveParameter::CurveParameter(float x0, float x1, float y[11]) {
    this->x0 = x0;
    this->x1 = x1;
    memcpy(this->y, y, sizeof(float) * 11);
}

float CurveParameter::operator()(float x) {
    float pct = (x - x0) / (x1 - x0) * 10.0f;
    pct = fminf(pct, 10.0f);
    pct = fmaxf(pct, 0.0f);

    int i = (int) floorf(pct);
    int j = (int) ceilf(pct);

    float a = pct - (float) i;
    float yr = ((1-a) * y[i]) + (a * y[j]);
    return yr;
}
