#ifndef VCU_CORE_STOMPP_H
#define VCU_CORE_STOMPP_H

#include "VcuParameters.h"


typedef struct StomppInput {
    float apps; // apps pedal travel (%)
    float bse; // brake pressure (psi)
} StomppInput;


typedef struct StomppOutput {
    bool ok;
} StomppOutput;


class Stompp {
public:
    void reset();
    void toggleStompp();
    void setParameters(VcuParameters* params) {};
    void evaluate(VcuParameters *params, StomppInput *input, StomppOutput *output, float deltaTime);
private:
    bool stomppActive = false;
};


#endif //VCU_CORE_STOMPP_H
