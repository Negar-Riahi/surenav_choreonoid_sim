#pragma once

#include "headers/DCM.h"
#include "headers/Link.h"
#include "headers/PID.h"

using namespace std;

class Robot{
    friend class Surena;
    public:
        Robot();

        vector<double> spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time);
        vector<double> spinOffline();

    private:

        DCMPlanner* trajectoryPlanner_;

        vector<_Link> joints_;

        PID* DCMController_;
        PID* CoMController_;

        void doIK(MatrixXd pelvisP, Matrix3d pelvisR, MatrixXd leftAnkleP, Matrix3d leftAnkleR, MatrixXd rightAnkleP, Matrix3d rightAnkleR);
        double* geometricIK(MatrixXd p1, MatrixXd r1, MatrixXd p7, MatrixXd r7, bool isLeft);

        Matrix3d Rroll(double phi);
        Matrix3d RPitch(double theta);
};
