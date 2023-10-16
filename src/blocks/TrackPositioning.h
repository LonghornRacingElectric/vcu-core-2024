#ifndef VCU_CORE_TRACKPOSITIONING_H
#define VCU_CORE_TRACKPOSITIONING_H


#include "VcuParameters.h"
#include "util/Structs.h"
#include "util/math/Matrix.h"

typedef struct TrackPositioningInput {
    double gpsLat; // degrees
    double gpsLong; // degrees
    float gpsSpeed; // knots
    float gpsHeading; // degrees

    xyz imu1Accel; // accel x, y, z (m/s^2)
    xyz imu2Accel;
    xyz imu3Accel;
    xyz imu1Gyro; // gyro x, y, z (rad/s)
    xyz imu2Gyro;
    xyz imu3Gyro;

    float wheelSpeedFl; // wheel speed (rad/s)
    float wheelSpeedFr;
    float wheelSpeedBl;
    float wheelSpeedBr;

    float wheelAngleFl; // wheel angle (rad), left turn is +
    float wheelAngleFr;
    float wheelAngleBl;
    float wheelAngleBr;
} TrackPositioningInput;

typedef struct TrackPositioningState {
    Matrix state;
    Matrix covariance;
} TrackPositioningState;

typedef struct TrackPositioningOutput {
    xyz vehicleDisplacement;
    xyz vehicleVelocity;
    xyz vehicleAcceleration;
} TrackPositioningOutput;


class TrackPositioning {
private:
    TrackPositioningState systemState;
    Matrix stateMatrix;

    // process covariance matrix
    Matrix P;

    // computational Identity matrix
    Matrix A;
    bool isInitialState = true;
    float t{};

public:
    void setParameters(VcuParameters* params) {};
    void evaluate(VcuParameters *params, TrackPositioningInput *input, TrackPositioningOutput *output, float deltaTime);
};


#endif //VCU_CORE_TRACKPOSITIONING_H
