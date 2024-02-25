
#include "TrackPositioning.h"
#include <vector>
#include "util/math/Position.h"
//#include <Eigen/Dense>

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
                {initialPositions.x}, // positional state
                {(float) input->gpsSpeed, 0} // velocity state
        };

        stateMatrix = *new Matrix(initialStateVector);

        t = 0;
        control = Matrix(3, 1);
        control.set(0, 0, 0);
        control.set(1, 0, 0);
        control.set(2, 0, 0);

        covariance = Matrix::getIdentityMatrix(3*2);

        filter = *new KalmanFilter(3 * 2, 0, stateMatrix, control, covariance);

        isInitialState = false;
    }


    // TODO -- IF WE ALREADY HAVE INITIAL VALUES, RUN CALCULATIONS WITH KALMAN FILTER AND MATRIX CLASS

    control.set(0,0, input->imu1Accel.x);
    control.set(1,0, input->imu1Accel.y);
    control.set(2,0, input->imu1Accel.z);

    // FILTER.state() should be a GPS State instead.
    filter.update(filter.state(), control, deltaTime);

    output->vehicleDisplacement = {filter.state().get(0,0), filter.state().get(0,1), filter.state().get(0,2)}; // m
    output->vehicleVelocity = {filter.state().get(0,3), filter.state().get(0,4), filter.state().get(0,5)}; // m/s
    output->vehicleAcceleration = {input->imu1Accel.x, input->imu1Accel.y, input->imu1Accel.z}; // m/s^2

    t += deltaTime;
}

