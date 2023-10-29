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
    float appsLowPassFilterTimeConstant = 0.000f; // (s)
    float appsImplausibilityTime = 0.100f; // (s)
    float appsPlausibilityRange = 0.10f; // (%)
    float apps1VoltageMin = 1.0f; // voltage corresponding to 0% pedal travel (V)
    float apps1VoltageMax = 4.0f; // voltage corresponding to 100% pedal travel (V)
    float apps2VoltageMin = 0.5f; // voltage corresponding to 0% pedal travel (V)
    float apps2VoltageMax = 2.0f; // voltage corresponding to 100% pedal travel (V)
    float appsDeadZonePct = 0.05f; // (%) of travel that is 0 or 100

    // ===== BSE PARAMETERS =====
    float bseLowPassFilterTimeConstant = 0.000f; // (s)
    float bseImplausibilityTime = 0.100f; // (s)
    float bseVoltageMin = 0.5f; // voltage corresponding to 0% brake pressure (V)
    float bseVoltageMax = 4.5f; // voltage corresponding to 100% brake pressure (V)
    float bsePressureMin = 0.0f; // minimum brake pressure (psi)
    float bsePressureMax = 1000.0f; // maximum brake pressure (psi)

    // ===== STOMPP PARAMETERS =====
    float stomppMechanicalBrakesThreshold = 100.0f; // minimum pressure to count as mechanical brakes engaged for EV 4.7 (psi)
    float stomppAppsCutoffThreshold = 0.25f; // apps pedal travel where motor power needs to be cut off (%)
    float stomppAppsRecoveryThreshold = 0.05f; // apps pedal travel where motor power can be restored (%)

    // ===== TORQUE MAP PARAMETERS =====
    CurveParameter mapPedalToTorqueRequest = CurveParameter(1.0f, 230.0f); // torque request (Nm) as a function of pedal travel (%)
    CurveParameter mapDerateMotorTemp; // allowed torque (%) as a function of motor temperature (deg C)
    CurveParameter mapDerateInverterTemp; // allowed torque (%) as a function of inverter temperature (deg C)
    CurveParameter mapDerateBatteryTemp; // allowed torque (%) as a function of battery temperature (deg C)
    CurveParameter mapDerateBatterySoc; // allowed torque (%) as a function of battery SoC (%)

    // ===== PRNDL PARAMETERS =====
    float prndlBrakeToStartThreshold = 100.0f; // minimum pressure required on the brakes to go into drive (psi)
    float prndlBuzzerDuration = 2.0f; // how long the buzzer buzzes (s)
    float prndlSwitchDebounceDuration = 0.100f; // how long a digital high/low must be sustained to be considered (s)

    // ==== TRACTION CONTROL PARAMETERS =====
    // tbd


    // ==== STEERING PARAMETERS ====
    CurveParameter steeringWheelToOuterWheel;
    CurveParameter steeringWheelToInnerWheel;
    float steeringPotMaxVoltage = 3.3f;
    float steeringWheelMaxAngle = 90.0f;


} VcuParameters;

#endif //VCU_CORE_VCUPARAMETERS_H
