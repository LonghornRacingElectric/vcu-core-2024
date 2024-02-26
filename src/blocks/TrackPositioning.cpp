
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
        initialLocation = {input->gpsLat, input->gpsLong};
        isInitialState = false;
    }


    // TODO -- IF WE ALREADY HAVE INITIAL VALUES, RUN CALCULATIONS WITH KALMAN FILTER AND MATRIX CLASS

    SquareDist displacement = Position::unitDistanceBetween(initialLocation.lng, initialLocation.lat, input->gpsLong, input->gpsLat);

    control.set(0,0, input->imu1Accel.x);
    control.set(1,0, input->imu1Accel.y);
    control.set(2,0, input->imu1Accel.z);

    // FILTER.state() should be a GPS State instead.
    filter.update(control, deltaTime);

    output->vehicleDisplacement = {filter.getState().get(0,0), filter.getState().get(0,1), filter.getState().get(0,2)}; // m
    output->vehicleVelocity = {filter.getState().get(0,3), filter.getState().get(0,4), filter.getState().get(0,5)}; // m/s
    output->vehicleAcceleration = {input->imu1Accel.x, input->imu1Accel.y, input->imu1Accel.z}; // m/s^2

    t += deltaTime;
}

