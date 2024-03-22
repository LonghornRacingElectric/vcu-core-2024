//
// Created by Dhairya Gupta on 10/15/23.
//

#include <gtest/gtest.h>
#include <util/math/Position.h>

#include <iostream>

#include "util/filters/ExtendedKalmanFilter.h"

TEST(Kalman, ExtendedKalmanFilter) {
  ExtendedKalmanFilter testFilter;

  ControlState control = {0, 1, 0};
  VehicleState gpsEstimate = {0, 0, 0, 0, 0};
  float delta_t = 0.003f;

  // drive 10 seconds at 1m/s in the Y direction (car-centric forward)
  for (int i = 0; i < 3330; i++) {
    testFilter.update(control, gpsEstimate, delta_t);
  }

  // moved a total of 50 meters
  std::cout << "\nAFTER STRAIGHT\n";
  std::cout << "X: " << testFilter.getState().x << "\n";
  std::cout << "Y: " << testFilter.getState().y << "\n";
  std::cout << "V_X: " << testFilter.getState().v_x << "\n";
  std::cout << "V_Y: " << testFilter.getState().v_y << "\n";
  std::cout << "THETA: " << testFilter.getState().theta << "\n";


  // make right turn
  control.a_x = -20.0;
  control.v_theta = Position::degreesToRadians(90);

  for (int i = 0; i < 333; i++) {
    testFilter.update(control, gpsEstimate, delta_t);
  }

  std::cout << "\nAFTER TURN\n";
  std::cout << "X: " << testFilter.getState().x << "\n";
  std::cout << "Y: " << testFilter.getState().y << "\n";
  std::cout << "V_X: " << testFilter.getState().v_x << "\n";
  std::cout << "V_Y: " << testFilter.getState().v_y << "\n";
  std::cout << "THETA: " << testFilter.getState().theta << "\n";

  control.a_x = 0.0;
  control.a_y = -10.0;
  control.v_theta = Position::degreesToRadians(0);

  for (int i = 0; i < 333; i++) {
    testFilter.update(control, gpsEstimate, delta_t);
  }

  std::cout << "\nAFTER BRAKING\n";
  std::cout << "X: " << testFilter.getState().x << "\n";
  std::cout << "Y: " << testFilter.getState().y << "\n";
  std::cout << "V_X: " << testFilter.getState().v_x << "\n";
  std::cout << "V_Y: " << testFilter.getState().v_y << "\n";
  std::cout << "THETA: " << testFilter.getState().theta << "\n";
}