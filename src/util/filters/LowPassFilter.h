#ifndef VCU_CORE_LOWPASSFILTER_H
#define VCU_CORE_LOWPASSFILTER_H

/**
 * Digital Low Pass Filter: http://techteach.no/simview/lowpass_filter/doc/filter_algorithm.pdf
 */
class LowPassFilter {
private:
    float defaultValue;
    float timeConstant;
    float accumulator;
public:
    LowPassFilter(float defaultValue, float timeConstant);
    void add(float value, float deltaTime);
    float get();
    void reset();
};


#endif //VCU_CORE_LOWPASSFILTER_H
