
#include <gtest/gtest.h>
#include "VcuModel.h"

TEST(WheelMagnets, Wheel) {
    VcuParameters params;
    WheelMagnets wheelMagnets;

    params.appsLowPassFilterTimeConstant = 0.0f;
    params.apps1VoltageMin = 1.0f;
    params.apps1VoltageMax = 4.0f;
    params.apps2VoltageMin = 0.5f;
    params.apps2VoltageMax = 2.0f;
    params.appsDeadZonePct = 0.05f;
    params.appsPlausibilityRange = 0.10f;
    params.appsImplausibilityTime = 0.100f;

    params.bseLowPassFilterTimeConstant = 0.005f;
    params.bseImplausibilityTime = 0.1f;
    params.bseVoltageMin = 0.5f;
    params.bseVoltageMax = 4.5f;

    params.prndlBrakeToStartThreshold = 0.1f;
    params.prndlSwitchDebounceDuration = 0.05f;
    params.prndlBuzzerDuration = 1.0f;

    params.mapPedalToTorqueRequest = CurveParameter(1.0f, 230.0f);

    wheelMagnets.setParameters(&params);
    wheelMagnets.evaluate(params);
}