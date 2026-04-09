#include <gtest/gtest.h>
#include <cmath>

#include "VcuModel.h"

namespace {
constexpr float kPi = 3.14159265f;
}

TEST(WheelMagnets, EstimatesWheelSpeedFromMagneticWaveform) {
    VcuParameters params;
    WheelMagnets wheelMagnets;

    params.wheelMagnetLpfTimeConstant = 0.030f;

    wheelMagnets.setParameters(&params);
    WheelMagnetsOutput output = {0,0,0,0};
    WheelMagnetsInput input = {0,0,0,0, true, true, true, true};
    float trueAngularVelocity = 12.0f;
    float dt = 0.003f;
    float amplitude = 0.9f;
    float baseline = 1.0f;

    for(int i=0; i < 1200; i++){
            float time = i * dt;
            float wave = baseline - (amplitude * (0.5f + 0.5f * std::sin(3.0f * trueAngularVelocity * time)));
            float noise = 0.05f * std::sin(11.0f * time);
            input.wheelMagneticFieldFr = wave + noise;
            input.wheelMagneticFieldFl = wave + noise;
            input.wheelMagneticFieldBl = wave + noise;
            input.wheelMagneticFieldBr = wave + noise;
            wheelMagnets.evaluate(&params, &input, &output, dt);
    }

    EXPECT_NEAR(output.wheelSpeedFl, trueAngularVelocity, 2.0f);
}

TEST(WheelMagnets, TracksSlowWheelSpeedWithoutDroppingToZero) {
    VcuParameters params;
    WheelMagnets wheelMagnets;

    params.wheelMagnetLpfTimeConstant = 0.050f;

    wheelMagnets.setParameters(&params);
    WheelMagnetsOutput output = {0,0,0,0};
    WheelMagnetsInput input = {0,0,0,0, true, true, true, true};
    float trueAngularVelocity = 8.0f;
    float dt = 0.003f;
    float amplitude = 1.2f;

    for(int i=0; i < 3000; i++){
            float time = i * dt;
            float wave = amplitude * std::sin(3.0f * trueAngularVelocity * time);
            float ripple = 0.08f * std::sin(19.0f * time);
            input.wheelMagneticFieldFr = wave + ripple;
            input.wheelMagneticFieldFl = wave + ripple;
            input.wheelMagneticFieldBl = wave + ripple;
            input.wheelMagneticFieldBr = wave + ripple;
            wheelMagnets.evaluate(&params, &input, &output, dt);
    }

    EXPECT_NEAR(output.wheelSpeedFl, trueAngularVelocity, 1.0f);
    EXPECT_GT(output.wheelSpeedFl, 1.0f);

}

TEST(WheelMagnets, EstimatesWheelSpeedFromOffsetDropoutSignal) {
    VcuParameters params;
    WheelMagnets wheelMagnets;

    params.wheelMagnetLpfTimeConstant = 0.050f;

    wheelMagnets.setParameters(&params);
    WheelMagnetsOutput output = {0,0,0,0};
    WheelMagnetsInput input = {0,0,0,0, true, true, true, true};
    float trueAngularVelocity = 12.0f;
    float dt = 0.003f;
    float amplitude = 0.90f;
    float baseline = 1.0f;

    for (int i = 0; i < 1800; i++) {
        float time = i * dt;
        float field = baseline - (amplitude * (0.5f + 0.5f * std::sin(3.0f * trueAngularVelocity * time)));
        field += 0.04f * std::sin(13.0f * time);
        if ((i % 240) == 0) {
            field = baseline;
        }
        input.wheelMagneticFieldFr = field;
        input.wheelMagneticFieldFl = field;
        input.wheelMagneticFieldBl = field;
        input.wheelMagneticFieldBr = field;
        wheelMagnets.evaluate(&params, &input, &output, dt);
    }

    EXPECT_NEAR(output.wheelSpeedFl, trueAngularVelocity, 2.5f);
    EXPECT_GT(output.wheelSpeedFl, 1.0f);
}

TEST(WheelMagnets, RejectsRestingNoiseInsteadOfInventingWheelSpeed) {
    VcuParameters params;
    WheelMagnets wheelMagnets;

    params.wheelMagnetLpfTimeConstant = 0.050f;

    wheelMagnets.setParameters(&params);
    WheelMagnetsOutput output = {0,0,0,0};
    WheelMagnetsInput input = {0,0,0,0, true, true, true, true};
    float dt = 0.003f;
    float maxObservedSpeed = 0.0f;

    for (int i = 0; i < 4000; i++) {
        float time = i * dt;
        float noise = 0.10f * std::sin(17.0f * time)
                    + 0.06f * std::sin(41.0f * time + 0.7f)
                    + 0.03f * std::sin(83.0f * time + 1.1f);
        input.wheelMagneticFieldFr = noise;
        input.wheelMagneticFieldFl = noise;
        input.wheelMagneticFieldBl = noise;
        input.wheelMagneticFieldBr = noise;
        wheelMagnets.evaluate(&params, &input, &output, dt);
        maxObservedSpeed = std::max(maxObservedSpeed, output.wheelSpeedFl);
    }

    EXPECT_LT(maxObservedSpeed, 2.0f);
    EXPECT_LT(output.wheelSpeedFl, 0.2f);
}
