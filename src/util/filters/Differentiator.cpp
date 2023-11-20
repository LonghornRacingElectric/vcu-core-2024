#include "Differentiator.h"

Differentiator::Differentiator() {
    this->last = 0.0f;
}

void Differentiator::reset() {
    this->last = 0.0f;
}

float Differentiator::get(float value, float deltaTime) {
    float dx_dt = (value - this->last) / deltaTime;
    this->last = value;
    return dx_dt;
}
