#ifndef VCU_CORE_BSEPROCESSOR_H
#define VCU_CORE_BSEPROCESSOR_H

typedef struct BseProcessorInput {
    float bse1;
    float bse2;
} BseProcessorInput;

typedef struct BseProcessorOutput {
    float bse;
    bool plausible;
}

class BseProcessor {
public:
    void evaluate() // TODO
};


#endif //VCU_CORE_BSEPROCESSOR_H
