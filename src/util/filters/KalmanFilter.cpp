//
// Created by Dhairya Gupta on 10/1/23.
//

#include "KalmanFilter.h"
#include <iostream>

KalmanFilter::KalmanFilter() {
    initialized = false;
    n = 0;
}

KalmanFilter::KalmanFilter(int variables, float t0, const Matrix &x0, const Matrix &u0, const Matrix &covariance) {
    n = variables;
    initialized = true;
    init(variables, t0, x0, u0, covariance);
}


void KalmanFilter::init(int variables, float t0, const Matrix &x0, const Matrix &u0, const Matrix &covariance)  {
    n = variables;

    stateTransition = Matrix::getIdentityMatrix(n * 2);
    for(int row = 0; row < stateTransition.rows(); row++) {
        for(int col = 1 + row; col < stateTransition.columns(); col += n) {
            stateTransition.set(row, col, t0);
        }
    }

    controlTransition = Matrix(n * 2, n);

    processCovariance = covariance;
//    processCovariance = Matrix(variables * 2, variables * 2);
    R = Matrix(variables * 2, variables * 2);

    for(int row = 0; row < processCovariance.rows(); row++) {
        processCovariance.set(row, row, covariance.get(row,row));
        R.set(row, row, covariance.get(row,row));
    }

    x_hat = x0;

    u = u0;

    initialized = true;
}

void KalmanFilter::update(const Matrix &y, const Matrix &control, float dt) {
    if(!initialized) {
        throw std::invalid_argument("Cannot invoke update without initialized Kalman Filter");
    }

    for(int row = 0; row < stateTransition.rows(); row++) {
        for(int col = n + row; col < stateTransition.columns(); col += n) {
            stateTransition.set(row, col, dt);
        }
    }

    /** Update the acceleration control transition matrix */
    for(int row = 0; row < n; row++) {
        controlTransition.set(row, row, 0.5f * (dt * dt));
    }

    for(int col = n; col < controlTransition.rows(); col++) {
        controlTransition.set(col, col - n, dt);
    }

//    std::cout << "\nCONTROL TRANSITION\n" + controlTransition.toString();

    x_hat_new = stateTransition * x_hat + controlTransition * u;

//    std::cout << "\nX HAT NEW\n" + x_hat_new.toString();

    u = control;

//    std::cout << "\nU\n" + u.toString();
//
//    std::cout << "\nSTATE TRANSITION\n" + stateTransition.toString();

    processCovariance = stateTransition * processCovariance * stateTransition.getTranspose();

    // remove extraneous values
    for(int row = 0; row < processCovariance.rows(); row++) {
        for(int col = 0; col < processCovariance.columns(); col++) {
            if(col != row && processCovariance.get(row, col) != 0) {
                processCovariance.set(row,col,0);
            }
        }
    }

//    std::cout << "\nPROCESS COVARIANCE\n" + processCovariance.toString();
//    std::cout << "\nR\n" + R.toString();

    Matrix kalmanGain = processCovariance / (processCovariance + R);

//    std::cout << "\nKALMAN GAIN\n" + kalmanGain.toString();
    Matrix H = Matrix::getIdentityMatrix(n*2);

//    std::cout << "\nH\n" + H.toString();

    x_hat = x_hat_new + kalmanGain * (y - H * x_hat_new);

//    std::cout << "\nX HAT\n" + x_hat.toString();

//    std::cout << "\nKALMAN GAIN X H\n" + (kalmanGain*H).toString();

    processCovariance = (H - (kalmanGain * H)) * processCovariance;

//    std::cout << "\nPROCESS COVARIANCE 2\n" + processCovariance.toString();
}