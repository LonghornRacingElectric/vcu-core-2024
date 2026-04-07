#include <gtest/gtest.h>
#include <cmath>

#include "VcuModel.h"

TEST(WheelMagnets, EstimatesWheelSpeedFromMagneticWaveform) {
    VcuParameters params;
    WheelMagnets wheelMagnets;

    params.wheelMagnetLpfTimeConstant = 0.030f;

    wheelMagnets.setParameters(&params);
    WheelMagnetsOutput output = {0,0,0,0};
    WheelMagnetsInput input = {0,0,0,0, true, true, true, true};
    float trueAngularVelocity = 100.0f;
    float dt = 0.003f;
    float amplitude = 15.0f;

    for(int i=0; i < 1200; i++){
            float time = i * dt;
            float wave = amplitude * std::sin(3.0f * trueAngularVelocity * time);
            float noise = 0.35f * std::sin(37.0f * time);
            input.wheelMagneticFieldFr = wave + noise;
            input.wheelMagneticFieldFl = wave + noise;
            input.wheelMagneticFieldBl = wave + noise;
            input.wheelMagneticFieldBr = wave + noise;
            wheelMagnets.evaluate(&params, &input, &output, dt);
    }

    EXPECT_NEAR(output.wheelSpeedFl, trueAngularVelocity, 4.0f);
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
