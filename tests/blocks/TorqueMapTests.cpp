#include <gtest/gtest.h>
#include "VcuParameters.h"
#include "blocks/TorqueMap.h"

namespace {

CurveParameter constantEfficiencyCurve(float efficiency) {
    float values[11] = {
        efficiency, efficiency, efficiency, efficiency, efficiency, efficiency,
        efficiency, efficiency, efficiency, efficiency, efficiency
    };
    return CurveParameter(0.0f, 5500.0f, values);
}

void primeOpenCircuitVoltageEstimate(TorqueMap *torqueMap,
                                     VcuParameters *params,
                                     TorqueMapOutput *output,
                                     float batteryVoltage) {
    TorqueMapInput input = {
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        100.0f,
        batteryVoltage,
        0.0f,
    };
    for (int i = 0; i < 5; i++) {
        torqueMap->evaluate(params, &input, output, 10.0f);
    }
}

} // namespace

TEST(TorqueMap, FeedforwardLeavesPedalLimitAtLowSpeed) {
    VcuParameters params;
    params.mapPowerLimit = 70000.0f;
    params.mapPowerLimitMotorEfficiency = constantEfficiencyCurve(0.95f);
    params.mapPowerLimit_kP = 0.0f;
    params.mapPowerLimit_kI = 0.0f;

    TorqueMap torqueMap;
    TorqueMapOutput output{};
    torqueMap.setParameters(&params);
    primeOpenCircuitVoltageEstimate(&torqueMap, &params, &output, 500.0f);

    TorqueMapInput input = {
        1.0f,
        0.0f,
        500.0f,
        0.0f,
        0.0f,
        100.0f,
        500.0f,
        0.0f,
    };

    torqueMap.evaluate(&params, &input, &output, 0.01f);
    EXPECT_NEAR(output.torqueRequest, 230.0f, 0.5f);
}

TEST(TorqueMap, PedalRequestsPercentageOfAvailableTorque) {
    VcuParameters params;
    params.mapPowerLimit = 70000.0f;
    params.mapPowerLimitMotorEfficiency = constantEfficiencyCurve(1.0f);
    params.mapPowerLimit_kP = 0.0f;
    params.mapPowerLimit_kI = 0.0f;

    TorqueMap torqueMap;
    TorqueMapOutput output{};
    torqueMap.setParameters(&params);
    primeOpenCircuitVoltageEstimate(&torqueMap, &params, &output, 520.0f);

    TorqueMapInput input = {
        0.5f,
        0.0f,
        3000.0f,
        0.0f,
        0.0f,
        100.0f,
        520.0f,
        0.0f,
    };

    torqueMap.evaluate(&params, &input, &output, 0.01f);
    EXPECT_NEAR(output.torqueRequest, 111.4f, 0.5f);
}

TEST(TorqueMap, FeedforwardCapsTorqueAtHigherSpeed) {
    VcuParameters params;
    params.mapPowerLimit = 70000.0f;
    params.mapPowerLimitMotorEfficiency = constantEfficiencyCurve(1.0f);
    params.mapPowerLimit_kP = 0.0f;
    params.mapPowerLimit_kI = 0.0f;

    TorqueMap torqueMap;
    TorqueMapOutput output{};
    torqueMap.setParameters(&params);
    primeOpenCircuitVoltageEstimate(&torqueMap, &params, &output, 520.0f);

    TorqueMapInput input = {
        1.0f,
        0.0f,
        3000.0f,
        0.0f,
        0.0f,
        100.0f,
        520.0f,
        0.0f,
    };

    torqueMap.evaluate(&params, &input, &output, 0.01f);
    EXPECT_NEAR(output.torqueRequest, 222.8f, 0.5f);
}

TEST(TorqueMap, ExponentialPedalSoftensLowPedalTorque) {
    VcuParameters params;
    params.mapPowerLimit = 70000.0f;
    params.mapPowerLimitMotorEfficiency = constantEfficiencyCurve(1.0f);
    params.mapPowerLimit_kP = 0.0f;
    params.mapPowerLimit_kI = 0.0f;
    params.mapPedalExponentialFactor = 2.0f;

    TorqueMap torqueMap;
    TorqueMapOutput output{};
    torqueMap.setParameters(&params);
    primeOpenCircuitVoltageEstimate(&torqueMap, &params, &output, 500.0f);

    TorqueMapInput input = {
        0.5f,
        0.0f,
        500.0f,
        0.0f,
        0.0f,
        100.0f,
        500.0f,
        0.0f,
    };

    torqueMap.evaluate(&params, &input, &output, 0.01f);
    EXPECT_NEAR(output.torqueRequest, 61.9f, 0.5f);
}

TEST(TorqueMap, TrimIsClampedAroundFeedforwardEstimate) {
    VcuParameters params;
    params.mapPowerLimit = 70000.0f;
    params.mapPowerLimitMotorEfficiency = constantEfficiencyCurve(1.0f);
    params.mapPowerLimit_kP = 0.01f;
    params.mapPowerLimit_kI = 0.0f;
    params.mapPowerLimitTrimLimit = 20.0f;

    TorqueMap torqueMap;
    TorqueMapOutput output{};
    torqueMap.setParameters(&params);
    primeOpenCircuitVoltageEstimate(&torqueMap, &params, &output, 520.0f);

    TorqueMapInput input = {
        1.0f,
        0.0f,
        3000.0f,
        0.0f,
        0.0f,
        100.0f,
        520.0f,
        150.0f,
    };

    torqueMap.evaluate(&params, &input, &output, 0.01f);
    EXPECT_FLOAT_EQ(output.feedbackTorque, -20.0f);
    EXPECT_NEAR(output.torqueRequest, 202.8f, 0.5f);
}

TEST(TorqueMap, DerivativeTrimSuppressesFastPowerRamp) {
    VcuParameters params;
    params.mapPowerLimit = 70000.0f;
    params.mapPowerLimitMotorEfficiency = constantEfficiencyCurve(1.0f);
    params.mapPowerLimitMeasuredPowerLpfTimeConstant = 0.0f;
    params.mapPowerLimit_kP = 0.0f;
    params.mapPowerLimit_kI = 0.0f;
    params.mapPowerLimit_kD = 0.0002f;
    params.mapPowerLimitTrimLimit = 20.0f;

    TorqueMap torqueMap;
    TorqueMapOutput output{};
    torqueMap.setParameters(&params);
    primeOpenCircuitVoltageEstimate(&torqueMap, &params, &output, 520.0f);

    TorqueMapInput baseInput = {
        1.0f,
        0.0f,
        3000.0f,
        0.0f,
        0.0f,
        100.0f,
        520.0f,
        100.0f,
    };
    torqueMap.evaluate(&params, &baseInput, &output, 0.01f);

    TorqueMapInput rampInput = {
        1.0f,
        0.0f,
        3000.0f,
        0.0f,
        0.0f,
        100.0f,
        520.0f,
        140.0f,
    };
    torqueMap.evaluate(&params, &rampInput, &output, 0.01f);

    EXPECT_LT(output.feedbackD, -20.0f);
    EXPECT_FLOAT_EQ(output.feedbackTorque, -20.0f);
    EXPECT_NEAR(output.torqueRequest, 202.8f, 0.5f);
}
