#ifndef VCU_CORE_VCUPARAMETERS_H
#define VCU_CORE_VCUPARAMETERS_H

/**
 * These are the tunable values of the VCU.
 *
 * Every numerical value here should be a default placeholder. These numbers are ideally getting loaded from
 * nonvolatile memory and getting updated by Texas Tune (or are coming from the sim).
 */

typedef struct VcuParameters {
    float maxTorque = 230; // Nm
} VcuParameters;

#endif //VCU_CORE_VCUPARAMETERS_H
