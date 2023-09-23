#include "VcuModel.h"

void VcuModel::setParameters(VcuParameters *newParams) {
    this->params = newParams;
    this->appsProcessor.setParameters(newParams);
    this->bseProcessor.setParameters(newParams);
    this->stompp.setParameters(newParams);
    this->torqueMap.setParameters(newParams);
    this->tractionControl.setParameters(newParams);
    this->prndl.setParameters(newParams);
    this->steering.setParameters(newParams);
    this->trackPositioning.setParameters(newParams);
    this->drs.setParameters(newParams);
    this->softShutdown.setParameters(newParams);
}

void VcuModel::evaluate(VcuInput *vcuInput, VcuOutput *vcuOutput, float deltaTime) {

    appsProcessorInput = {
            vcuInput->apps1,
            vcuInput->apps2,
    };
    appsProcessor.evaluate(params, &appsProcessorInput, &appsProcessorOutput, deltaTime);

    bseProcessorInput = {
            vcuInput->bse1,
            vcuInput->bse2,
    };
    bseProcessor.evaluate(params, &bseProcessorInput, &bseProcessorOutput, deltaTime);

    stomppInput = {
            appsProcessorOutput.apps,
            bseProcessorOutput.bse,
    };
    stompp.evaluate(params, &stomppInput, &stomppOutput, deltaTime);

    torqueMapInput = {
            appsProcessorOutput.apps,
            vcuInput->motorTemp,
            vcuInput->inverterTemp,
            vcuInput->batteryTemp,
            vcuInput->batterySoc,
    };
    torqueMap.evaluate(params, &torqueMapInput, &torqueMapOutput, deltaTime);

    tractionControlInput = {
            torqueMapOutput.torqueRequest,
            vcuInput->wheelSpeedFl,
            vcuInput->wheelSpeedFr,
            vcuInput->wheelSpeedBl,
            vcuInput->wheelSpeedBr,
    };
    tractionControl.evaluate(params, &tractionControlInput, &tractionControlOutput, deltaTime);

    steeringInput = {
            vcuInput->steeringWheelPotVoltage,
    };
    steering.evaluate(params, &steeringInput, &steeringOutput, deltaTime);

    prndlInput = {
            vcuInput->driveSwitch,
            vcuInput->inverterReady,
            bseProcessorOutput.bse,
    };
    prndl.evaluate(params, &prndlInput, &prndlOutput, deltaTime);

    trackPositioningInput = {
            vcuInput->gpsLat,
            vcuInput->gpsLong,
            vcuInput->gpsSpeed,
            vcuInput->gpsHeading,

            vcuInput->imu1Accel,
            vcuInput->imu2Accel,
            vcuInput->imu3Accel,

            vcuInput->imu1Gyro,
            vcuInput->imu2Gyro,
            vcuInput->imu3Gyro,

            vcuInput->wheelSpeedFl,
            vcuInput->wheelSpeedFr,
            vcuInput->wheelSpeedBl,
            vcuInput->wheelSpeedBr,

            steeringOutput.wheelAngleFl,
            steeringOutput.wheelAngleFr,
            steeringOutput.wheelAngleBl,
            steeringOutput.wheelAngleBr,
    };
    trackPositioning.evaluate(params, &trackPositioningInput, &trackPositioningOutput, deltaTime);

    drsInput = {
            steeringOutput.steeringWheelAngle,
            trackPositioningOutput.vehicleVelocity,
    };
    drs.evaluate(params, &drsInput, &drsOutput, deltaTime);

    softShutdownInput = {
            appsProcessorOutput.ok,
            bseProcessorOutput.ok,
            stomppOutput.ok,
            prndlOutput.state,
            tractionControlOutput.regulatedTorqueRequest,
    };
    softShutdown.evaluate(params, &softShutdownInput, &softShutdownOutput, deltaTime);

    *vcuOutput = {
        softShutdownOutput.enableInverter,
        softShutdownOutput.inverterTorqueRequest,

        prndlOutput.state,
        prndlOutput.buzzer,

        drsOutput.enable,

        trackPositioningOutput.vehicleDisplacement,
        trackPositioningOutput.vehicleVelocity,
        trackPositioningOutput.vehicleAcceleration,

        !appsProcessorOutput.ok,
        !bseProcessorOutput.ok,
        !stomppOutput.ok,
        !steeringOutput.ok,
    };

}
