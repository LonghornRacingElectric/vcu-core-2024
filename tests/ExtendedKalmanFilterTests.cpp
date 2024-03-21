//
// Created by Dhairya Gupta on 10/15/23.
//

#include <gtest/gtest.h>
#include <util/math/Position.h>

#include <iostream>

#include "util/filters/ExtendedKalmanFilter.h"

TEST(Kalman, ExtendedKalmanFilter) {
    ExtendedKalmanFilter testFilter;

    ControlState control = {0, 1.002003, 0};
    VehicleState gpsEstimate = {0, 0, 0, 0, 0};
    float delta_t = 0.03;

    // drive 10 seconds at 1m/s in the Y direction (car-centric forward)
    for (int i = 0; i < 334; i++) {
        testFilter.update(control, gpsEstimate, delta_t);
    }

    // moved a total of 50 meters

    std::cout << "Control A_y: " << control.a_y << "\n";

        // start decelerating (3 seconds to get to 0), and turn 30 degrees clockwise each step
        control.a_y = -3.3;
        control.v_theta = Position::degreesToRadians(30);

        for (int i = 0; i < 100; i++) {
            testFilter.update(control, gpsEstimate, delta_t);
        }

        control.a_y = 1.2;
        control.v_theta = Position::degreesToRadians(0);

        for (int i = 0; i < 333; i++) {
            testFilter.update(control, gpsEstimate, delta_t);
        }

    std::cout << "\nFINAL STATE\n";
    std::cout << "X: " << testFilter.getState().x << "\n";
    std::cout << "Y: " << testFilter.getState().y << "\n";
    std::cout << "V_X: " << testFilter.getState().v_x << "\n";
    std::cout << "V_Y: " << testFilter.getState().v_y << "\n";
    std::cout << "THETA: " << testFilter.getState().theta << "\n";
}