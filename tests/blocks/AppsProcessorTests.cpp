
#include <gtest/gtest.h>
#include "VcuParameters.h"
#include "blocks/AppsProcessor.h"

TEST(AppsProcessor, AppsRulesCompliance) {
    AppsProcessor appsProcessor;
    AppsProcessorInput appsProcessorInput = {};
    AppsProcessorOutput appsProcessorOutput = {};

    VcuParameters vcuParameters = {};
    vcuParameters.apps1VoltageMin = 1.0f;
    vcuParameters.apps1VoltageMax = 4.0f;
    vcuParameters.apps2VoltageMin = 0.5f;
    vcuParameters.apps2VoltageMax = 2.0f;
    vcuParameters.appsPlausibilityRange = 0.1;

    // Test 0% pedal travel
    appsProcessorInput = {1.0f, 0.5f};
    appsProcessor.evaluate(&vcuParameters, &appsProcessorInput, &appsProcessorOutput, 1.0f);
    EXPECT_FLOAT_EQ(appsProcessorOutput.apps, 0.0f);
    EXPECT_TRUE(appsProcessorOutput.ok);

    // Test 50% pedal travel
    appsProcessorInput = {2.5f, 1.25f};
    appsProcessor.evaluate(&vcuParameters, &appsProcessorInput, &appsProcessorOutput, 1.0f);
    EXPECT_FLOAT_EQ(appsProcessorOutput.apps, 0.5f);
    EXPECT_TRUE(appsProcessorOutput.ok);

    // Test 100% pedal travel
    appsProcessorInput = {4.0f, 2.0f};
    appsProcessor.evaluate(&vcuParameters, &appsProcessorInput, &appsProcessorOutput, 1.0f);
    EXPECT_FLOAT_EQ(appsProcessorOutput.apps, 1.0f);
    EXPECT_TRUE(appsProcessorOutput.ok);

    // Test implausibility
    appsProcessorInput = {3.0f, 2.0f};
    appsProcessor.evaluate(&vcuParameters, &appsProcessorInput, &appsProcessorOutput, 1.0f);
    EXPECT_FALSE(appsProcessorOutput.ok);

    // Test out of bounds 1
    appsProcessorInput = {4.5f, 1.0f};
    appsProcessor.evaluate(&vcuParameters, &appsProcessorInput, &appsProcessorOutput, 1.0f);
    EXPECT_FALSE(appsProcessorOutput.ok);

    // Test out of bounds 2
    appsProcessorInput = {3.0f, 0.2f};
    appsProcessor.evaluate(&vcuParameters, &appsProcessorInput, &appsProcessorOutput, 1.0f);
    EXPECT_FALSE(appsProcessorOutput.ok);

    // Test clock function
    appsProcessorInput = {4.0f, 0.5f};
    appsProcessor.evaluate(&vcuParameters, &appsProcessorInput, &appsProcessorOutput, 0);
    EXPECT_FALSE(appsProcessorOutput.ok);
}
