#ifndef VCU_CORE_VCUPARAMETERS_H
#define VCU_CORE_VCUPARAMETERS_H

#include "util/parameters/CurveParameter.h"
#include "util/parameters/SurfaceParameter.h"

/**
 * These are the tunable values of the VCU.
 *
 * They can either be numbers (float), curves (CurveParameter) or surfaces/tables (SurfaceParameter).
 *
 * These parameters are ideally getting loaded from the VCU's nonvolatile memory
 * and getting updated by Texas Tune (or are coming from the sim).
 */

typedef struct VcuParameters {

    // ===== APPS PARAMETERS =====
    float appsLowPassFilterTimeConstant; // (s)
    float appsImplausibilityTime; // (s)
    float appsPlausibilityRange; // (%)
    float apps1VoltageMin; // voltage corresponding to 0% pedal travel (V)
    float apps1VoltageMax; // voltage corresponding to 100% pedal travel (V)
    float apps2VoltageMin; // voltage corresponding to 0% pedal travel (V)
    float apps2VoltageMax; // voltage corresponding to 100% pedal travel (V)
    float appsDeadZonePct; // (%) of travel that is 0 or 100

    // ===== BSE PARAMETERS =====
    float bseLowPassFilterTimeConstant; // (s)
    float bsePlausibilityRange; // (%)
    float bseVoltageMin; // voltage corresponding to 0% brake pressure (V)
    float bseVoltageMax; // voltage corresponding to 100% brake pressure (V)
    float bsePressureMin; // minimum brake pressure (psi)
    float bsePressureMax; // maximum brake pressure (psi)

    // ===== STOMPP PARAMETERS =====
    float stomppMechanicalBrakesThreshold; // minimum pressure to count as mechanical brakes engaged for EV 4.7 (psi)
    float stomppAppsCutoffThreshold; // apps pedal travel where motor power needs to be cut off (%)
    float stomppAppsRecoveryThreshold; // apps pedal travel where motor power can be restored (%)

    // ===== TORQUE MAP PARAMETERS =====
    CurveParameter mapPedalToTorqueRequest; // torque request (Nm) as a function of pedal travel (%)
    CurveParameter mapDerateMotorTemp; // allowed torque (%) as a function of motor temperature (deg C)
    CurveParameter mapDerateInverterTemp; // allowed torque (%) as a function of inverter temperature (deg C)
    CurveParameter mapDerateBatteryTemp; // allowed torque (%) as a function of battery temperature (deg C)
    CurveParameter mapDerateBatterySoc; // allowed torque (%) as a function of battery SoC (%)

    // ==== TRACTION CONTROL PARAMETERS =====
    // tbd

} VcuParameters;

#endif //VCU_CORE_VCUPARAMETERS_H
