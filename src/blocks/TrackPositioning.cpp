
#include "TrackPositioning.h"

#include <vector>

#include "util/math/Position.h"

/**
 * Dead reckoning! Use accelerometer, GPS, and potentially even wheel speed and angle data to
 * estimate the position of the car within the track. Probably use an Extended Kalman Filter, and
 * start with just accel and GPS.
 */

void TrackPositioning::evaluate(VcuParameters *params, TrackPositioningInput *input,
                                TrackPositioningOutput *output, float deltaTime) {
    // TODO implement
    if (isInitialState) {
        initialLocation = {input->gpsLat, input->gpsLong};
        filter = ExtendedKalmanFilter();
        control.a_x = 0;
        control.a_y = 0;
        control.v_theta = 0;
        isInitialState = false;
    }

    SquareDist displacement = Position::unitDistanceBetween(
        initialLocation.lng, initialLocation.lat, input->gpsLong, input->gpsLat);

    // TODO: pre-process each vector to account for the mount angle
    control.a_x = (input->imu1Accel.x + input->imu2Accel.x + input->imu3Accel.x) / 3.0f;
    control.a_y = (input->imu1Accel.y + input->imu2Accel.y + input->imu3Accel.y) / 3.0f;

    // TODO: where does the theta change come from -- check if this is correct.
    control.v_theta = input->imu1Gyro.x;

    gpsEstimate.x = displacement.x_displacement;
    gpsEstimate.y = displacement.y_displacement;
    gpsEstimate.theta = 90.0 - input->gpsHeading;

    // get the GPS speed for both directions
    gpsEstimate.v_x = input->gpsSpeed * sin(gpsEstimate.theta);
    gpsEstimate.v_y = input->gpsSpeed * cos(gpsEstimate.theta);

    filter.update(control, gpsEstimate, deltaTime);

    output->vehicleDisplacement = {filter.getState().x, filter.getState().y, 0};   // m
    output->vehicleVelocity = {filter.getState().v_x, filter.getState().v_y, 0};   // m/s
    output->vehicleAcceleration = {control.a_x, control.a_y, input->imu1Accel.z};  // m/s^2

    t += deltaTime;
}
