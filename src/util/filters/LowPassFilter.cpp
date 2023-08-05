#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float defaultValue, float timeConstant) {
    this->timeConstant = timeConstant;
    this->defaultValue = defaultValue;
    this->accumulator = defaultValue;
}

void LowPassFilter::reset() {
    this->accumulator = this->defaultValue;
}

void LowPassFilter::add(float value, float deltaTime) {
    float alpha = deltaTime / (timeConstant + deltaTime);
    accumulator = (accumulator * (1-alpha)) + (value * alpha);
}

float LowPassFilter::get() {
    return accumulator;
}
