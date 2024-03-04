//
// Created by Dhairya Gupta on 2/18/24.
//

#include "ExtendedKalmanFilter.h"

#include <cmath>
#include <iostream>

#include "util/math/Position.h"

/**
 * @brief Construct a new Extended Kalman Filter object
 *
 */
ExtendedKalmanFilter::ExtendedKalmanFilter() {
    jacobian = Matrix::getIdentityMatrix(5);
    covariance_estimate = Matrix(5, 5, 0);
    process_covariance = Matrix(5, 5, 0.000000000);
    observation_covariance = Matrix(5, 5, 0.000000f);

    x = 0, y = 0, v_x = 0, v_y = 0, theta = 0;
    state = Matrix(5, 1);
    innovation_state = Matrix(5, 1);
}

/**
 *  -------------------------
 * | Filter Prediction Steps |
 *  -------------------------
 */

/**
 * Predict step. Applies linear calculations to find new 'perfect' state of the system. Updates
 * class variables directly.
 * @param control the control state of the vehicle (IMU data in our case)
 * @param delta_t the time since the last update
 */
void ExtendedKalmanFilter::predictState(ControlState control, float delta_t) {
    // Gets the acceleration of the vehicle in terms of global coordinates (instead of vehicle
    // coordinates)

    PositionalState current_state = {control.a_x, control.a_y};
    PositionalState global_state_accel = getMathUnits(&current_state, theta);
    // PositionalState global_state = current_state;

    // Calculate new position based on previous velocity estimate
    double y_new = y + v_y * delta_t + ((1.0 / 2.0) * global_state_accel.y * (delta_t * delta_t));
    double x_new = x + v_x * delta_t + ((1.0 / 2.0) * global_state_accel.x * (delta_t * delta_t));
    double theta_new = control.v_theta * delta_t + theta;

    std::cout << "New Position: " << x_new << ", " << y_new << "\n";

    // Calculate new velocities
    double v_x_new = v_x + delta_t * global_state_accel.x;
    double v_y_new = v_y + delta_t * global_state_accel.y;

    // calculate the jacobian for the current iteration
    computeStateTransitionJacobian({delta_t, a_x, a_y, theta});

    // update the variables in the filter state.
    y = y_new;
    v_y = v_y_new;
    theta = theta_new;

    x = x_new;
    v_x = v_x_new;

    a_x = control.a_x;
    a_y = control.a_y;

    // update state matrix
    state.set(0, 0, x);
    state.set(1, 0, y);
    state.set(2, 0, v_x);
    state.set(3, 0, v_y);
    state.set(4, 0, theta);

    std::cout << "State: \n" << state.toString() << "\n";
}

Matrix ExtendedKalmanFilter::computeStateTransitionJacobian(JacobianVariables variables) {
    // Calculated using the partial derivatives of every derived value
    jacobian.set(0, 2, variables.delta_t);
    jacobian.set(1, 3, variables.delta_t);

    double sinTheta = sin(variables.theta);
    double cosTheta = sin(variables.theta);

    float x_partial = (1.0f / 2.0f) * (variables.delta_t * variables.delta_t) *
                      (-variables.a_x * sinTheta - variables.a_y * cosTheta);
    float y_partial = (1.0f / 2.0f) * (variables.delta_t * variables.delta_t) *
                      (variables.a_x * cosTheta - variables.a_y * sinTheta);

    float x_dot_partial =
        variables.delta_t * (-variables.a_x * sinTheta - variables.a_y * cosTheta);
    float y_dot_partial = variables.delta_t * (variables.a_x * cosTheta - variables.a_y * sinTheta);

    jacobian.set(0, 4, x_partial);
    jacobian.set(1, 4, y_partial);
    jacobian.set(2, 4, x_dot_partial);
    jacobian.set(3, 4, y_dot_partial);

    // std::cout << "Jacobian: \n" << jacobian.toString() << "\n";

    return jacobian;
}

Matrix ExtendedKalmanFilter::predictCovariance() {
    return jacobian * covariance_estimate * jacobian.getTranspose() + process_covariance;
}

/**
 *  -------------------------
 * |   State Update Steps   |
 *  -------------------------
 */

/**
 * @brief Main update method for the EKF. Updates the state estimate and covariance estimate. Access
 * updated state using the `getState()` method.
 *
 * @param u the control input (IMU data in our case)
 * @param z the measured (noisy) input estimate (GPS in our case)
 * @param delta_time the time since the last update
 */
void ExtendedKalmanFilter::update(ControlState u, VehicleState z, float delta_time) {
    // update state prediction and generate the jacobian matrix
    predictState(u, delta_time);

    // std::cout << "State: \n" << state.toString() << "\n";

    // Because we are assuming h(*) is insignificant, we can simply use z_k as y
    Matrix predicted_covariance = predictCovariance();

    // std::cout << "Predicted Covariance: \n" << predicted_covariance.toString() << "\n";

    // update the covariance estimate
    Matrix residual_covariance = getResidualCovariance();

    // std::cout << "Residual Covariance: \n" << residual_covariance.toString() << "\n";

    Matrix kalman_gain = getOptimalKalmanGain(residual_covariance);

    // std::cout << "Kalman Gain: \n" << kalman_gain.toString() << "\n";

    // GPS state into a matrix
    innovation_state.set(0, 0, z.x);
    innovation_state.set(1, 0, z.y);
    innovation_state.set(2, 0, z.v_x);
    innovation_state.set(3, 0, z.v_y);
    innovation_state.set(4, 0, z.theta);

    state = updateStateEstimate(kalman_gain, innovation_state);
    covariance_estimate = updateCovarianceEstimate(predicted_covariance);

    struct_state = {state.get(0, 0), state.get(1, 0), state.get(2, 0), state.get(3, 0),
                    state.get(4, 0)};
}

Matrix ExtendedKalmanFilter::getResidualCovariance() {
    return covariance_estimate + observation_covariance;
}

Matrix ExtendedKalmanFilter::getOptimalKalmanGain(const Matrix& residual_covariance) {
    // std::cout << "Kalman Gain Division : \n "
    //           << Matrix(5, 5, 1).divide(residual_covariance).toString() << "\n";
    // std::cout << "Kalman Gain Multiplication : \n "
    //           << (covariance_estimate * Matrix(5, 5, 1).divide(residual_covariance)).toString()
    //           << "\n";

    return covariance_estimate * Matrix(5, 5, 1).divide(residual_covariance);
}

Matrix ExtendedKalmanFilter::updateStateEstimate(const Matrix& kalman_gain, const Matrix& y) {
    // std::cout << "Kalman Gain * y : \n" << (kalman_gain * y).toString() << "\n";

    // std::cout << "State : \n" << state.toString() << "\n";
    // std::cout << "With State : \n" << (state + (kalman_gain * y)).toString() << "\n";
    return state + (kalman_gain * y);
}

Matrix ExtendedKalmanFilter::updateCovarianceEstimate(const Matrix& predicted_covariance) {
    return (Matrix::getIdentityMatrix(5) - jacobian) * predicted_covariance;
}

/**
 *  -------------------------
 * |     Helper Methods     |
 *  -------------------------
 */

/**
 * Calculates the global theta value based on the given theta value.
 *
 * @param theta The theta value in degrees.
 * @return The global theta value calculated as 90.0 minus the given theta value.
 */
double ExtendedKalmanFilter::getMathTheta(double theta) { return 90.0 - theta; }

/**
 * Converts a vehicle-local positional state to a global positional state.
 *
 * @param loc The local positional state to be converted.
 * @param theta The angle of rotation from the local coordinate system to the global coordinate
 * system in degrees
 * @return The converted global positional state.
 */
PositionalState ExtendedKalmanFilter::getMathUnits(PositionalState* loc, double theta) {
    double global_theta = Position::degreesToRadians(getMathTheta(theta));
    // double global_theta = theta;
    double global_y = loc->x * cos(global_theta) + loc->y * sin(global_theta);
    double global_x = loc->x * sin(global_theta) - loc->y * cos(global_theta);
    // double global_x = sqrt(pow(loc->x, 2) + pow(loc->y, 2)) * cos(global_theta);
    // double global_y = sqrt(pow(loc->x, 2) + pow(loc->y, 2)) * sin(global_theta);

    PositionalState newState = {global_x, global_y};

    std::cout << "Theta: " << theta << "\n";
    std::cout << "Global Theta: " << global_theta << "\n";
    std::cout << "New State: " << newState.x << ", " << newState.y << "\n";

    return newState;
}