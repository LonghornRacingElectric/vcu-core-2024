#ifndef VCU_CORE_VCUPARAMETERS_H
#define VCU_CORE_VCUPARAMETERS_H

#include "util/parameters/FunctionParameter2D.h"
#include "util/parameters/FunctionParameter3D.h"

/**
 * These are the tunable values of the VCU.
 *
 * They can either be numbers (float), curves (FunctionParameter2D) or surfaces/tables (FunctionParameter3D).
 *
 * These parameters are ideally getting loaded from the VCU's nonvolatile memory
 * and getting updated by Texas Tune (or are coming from the sim).
 */

typedef struct VcuParameters {

    // ===== APPS PARAMETERS =====
    float appsLowPassFilterTimeConstant; // (s)
    float appsPlausibilityRange; // (%)
    float apps1VoltageMin; // voltage corresponding to 0% pedal travel (V)
    float apps1VoltageMax; // voltage corresponding to 100% pedal travel (V)
    float apps2VoltageMin; // voltage corresponding to 0% pedal travel (V)
    float apps2VoltageMax; // voltage corresponding to 100% pedal travel (V)

    // ===== BSE PARAMETERS =====
    float bseLowPassFilterTimeConstant; // (s)
    float plausibilityRange; // (%)
    FunctionParameter2D bse1; // brake pressure (psi) as a function of voltage (V)
    FunctionParameter2D bse2;

    // ===== TORQUE MAP PARAMETERS =====
    FunctionParameter2D pedalToTorqueRequest; // torque request (Nm) as a function of pedal travel (%)
    // can be changed to a 3D function parameter to add RPM as an additional input axis

    // ==== TRACTION CONTROL PARAMETERS =====
    // tbd

} VcuParameters;

#endif //VCU_CORE_VCUPARAMETERS_H
