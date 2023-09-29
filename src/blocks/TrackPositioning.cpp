
#include "TrackPositioning.h"
#include <vector>
#include "util/math/Position.h"

/**
 * Dead reckoning! Use accelerometer, GPS, and potentially even wheel speed and angle data to estimate the position
 * of the car within the track. Probably use an Extended Kalman Filter, and start with just accel and GPS.
 */
void TrackPositioning::evaluate(VcuParameters *params, TrackPositioningInput *input, TrackPositioningOutput *output,
                                float deltaTime) {
    // TODO implement
    if(isInitialState) {
        // TODO -- SETUP INITIAL STATE WITH A NEW COVARIANCE MATRIX
        xyz initialPositions = Position::convertToXYZ(input->gpsLat, input->gpsLong);

        std::vector<std::vector<float>> initialStateVector {
                {initialPositions.x, initialPositions.y}, // positional state
                {(float) input->gpsSpeed} // velocity state
        };
        auto* initialState = new Matrix(initialStateVector);


        isInitialState = false;
    }

    // TODO -- IF WE ALREADY HAVE INITIAL VALUES, RUN CALCULATIONS WITH KALMAN FILTER AND MATRIX CLASS
    float xVelocity = input->gpsSpeed + ((input->imu2Accel.x +  input->imu1Accel.x)/2) * deltaTime;
    float yVelocity = input->gpsSpeed + ((input->imu2Accel.y +  input->imu1Accel.y)/2) * deltaTime;
    float zVelocity = input->gpsSpeed + ((input->imu2Accel.z +  input->imu1Accel.z)/2) * deltaTime;

    xyz displacement = {xVelocity * deltaTime, yVelocity * deltaTime, zVelocity * deltaTime};

    output->vehicleDisplacement = {0, 0, 0}; // m
    output->vehicleVelocity = {0, 0, 0}; // m/s
    output->vehicleAcceleration = {0, 0, 0}; // m/s^2
}

