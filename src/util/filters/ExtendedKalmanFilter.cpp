//
// Created by Dhairya Gupta on 2/18/24.
//

#include "ExtendedKalmanFilter.h"
#include <cmath>

/**
 * Predict step. Applies linear calculations to find new 'perfect' state of the system.
 * @param control
 * @param delta_t
 */
void ExtendedKalmanFilter::f(ControlState *control, float delta_t) {
    // Gets the acceleration of the vehicle in terms of global coordinates (instead of vehicle coordinates)

    PositionalState current_state = {control->a_x, control->a_y};
    PositionalState global_state = convertLocalToGlobal(&current_state, theta);

    // Calculate new position based on previous velocity estimate
    double y_new = y + v_y + ((1.0 / 2.0) * global_state.y * (delta_t * delta_t));
    double x_new = x + v_x + ((1.0 / 2.0) * global_state.x * (delta_t * delta_t));
    double theta_new = control->v_theta * delta_t + theta;

    // Calculate new velocities
    double v_x_new = v_x + delta_t * global_state.x;
    double v_y_new = v_y + delta_t * global_state.y;

    // update the variables in the filter state.
    y = y_new;
    v_y = v_y_new;
    theta = theta_new;

    x = x_new;
    v_x = v_x_new;
}

double ExtendedKalmanFilter::getGlobalTheta(double theta) {
    return 90.0-theta;
}

PositionalState ExtendedKalmanFilter::convertLocalToGlobal(PositionalState *loc, double theta) {
    double global_theta = getGlobalTheta(theta);
    double global_x = loc->x * cos(global_theta) - loc->y * sin(global_theta);
    double global_y = loc->x * sin(global_theta) + loc->y * cos(global_theta);

    PositionalState newState = {global_x, global_y};

    return newState;
}

  Matrix ExtendedKalmanFilter::evaluateJacobian(JacobianVariables variables) {
    // Calculated using the partial derivatives of every derived value
    jacobian.set(0, 2, variables.delta_t);
    jacobian.set(1, 3, variables.delta_t);

    float x_partial = (1.0f/2.0f) * (variables.delta_t * variables.delta_t) * (-variables.a_x * sin(variables.theta) - variables.a_y * cos(variables.theta));
    float y_partial = (1.0f/2.0f) * (variables.delta_t * variables.delta_t) * (variables.a_x * cos(variables.theta) - variables.a_y * sin(variables.theta));

    float x_dot_partial = variables.delta_t * (-variables.a_x * sin(variables.theta) - variables.a_y * cos(variables.theta));
    float y_dot_partial = variables.delta_t * (variables.a_x * cos(variables.theta) - variables.a_y * sin(variables.theta));

    jacobian.set(0, 4, x_partial);
    jacobian.set(1, 4, y_partial);
    jacobian.set(2, 4, x_dot_partial);
    jacobian.set(3, 4, y_dot_partial);

    return jacobian;
}

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    jacobian = Matrix::getIdentityMatrix(5);
    x = 0, y = 0, v_x = 0, v_y = 0, theta = 0;
    state = Matrix(5,1);
    state_prev = Matrix(5, 1);
}
