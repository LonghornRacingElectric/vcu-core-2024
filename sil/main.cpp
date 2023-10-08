#include <iostream>
#include "VcuModel.h"

using namespace std;

VcuParameters vcuParameters;
VcuInput vcuInput;
VcuOutput vcuOutput;

VcuModel vcuModel;

bool input_bool() {
    float x;
    cin >> x;
    return x != 0.0f;
}

xyz input_xyz() {
    float x, y, z;
    cin >> x;
    cin >> y;
    cin >> z;
    return xyz {x, y, z};
}

void input() {
    cin >> vcuInput.apps1;
    cin >> vcuInput.apps2;

    cin >> vcuInput.bse1;
    cin >> vcuInput.bse2;

    cin >> vcuInput.steeringWheelPotVoltage;

    cin >> vcuInput.wheelDisplacementFl;
    cin >> vcuInput.wheelDisplacementFr;
    cin >> vcuInput.wheelDisplacementBl;
    cin >> vcuInput.wheelDisplacementBr;

    cin >> vcuInput.motorTemp;
    cin >> vcuInput.inverterTemp;
    cin >> vcuInput.batteryTemp;

    cin >> vcuInput.batterySoc;
    vcuInput.inverterReady = input_bool();

    vcuInput.driveSwitch = input_bool();

    cin >> vcuInput.gpsLat;
    cin >> vcuInput.gpsLong;
    cin >> vcuInput.gpsSpeed;
    cin >> vcuInput.gpsHeading;

    vcuInput.imu1Accel = input_xyz();
    vcuInput.imu2Accel = input_xyz();
    vcuInput.imu3Accel = input_xyz();
    vcuInput.imu1Gyro = input_xyz();
    vcuInput.imu2Gyro = input_xyz();
    vcuInput.imu3Gyro = input_xyz();
}

void output_bool(bool b) {
    cout << (int)b << endl;
}

void output_xyz(xyz vec) {
    cout << vec.x << endl;
    cout << vec.y << endl;
    cout << vec.z << endl;
}

void output() {
    output_bool(vcuOutput.enableInverter);
    cout << vcuOutput.inverterTorqueRequest << endl;

    output_bool(vcuOutput.prndlState);
    output_bool(vcuOutput.r2dBuzzer);

    output_bool(vcuOutput.enableDragReduction);

    output_xyz(vcuOutput.vehicleDisplacement);
    output_xyz(vcuOutput.vehicleVelocity);
    output_xyz(vcuOutput.vehicleAcceleration);

    output_bool(vcuOutput.faultApps);
    output_bool(vcuOutput.faultBse);
    output_bool(vcuOutput.faultStompp);
    output_bool(vcuOutput.faultSteering);
}

int main() {
    vcuModel.setParameters(&vcuParameters);

    while(1) {
        input();
        vcuModel.evaluate(&vcuInput, &vcuOutput, 0.001f);
        output();
    }
}