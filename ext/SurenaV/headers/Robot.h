#pragma once

#include "headers/DCM.h"
#include "headers/Link.h"
#include "headers/PID.h"

using namespace std;

class Robot{
    public:
        Robot();

        vector<double> spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time);
        vector<double> spinOffline();

    private:

        DCMPlanner* trajectoryPlanner_;

        vector<_Link> joints_;

        PID* DCMController_;
        PID* CoMController_;

        void doIK(Vector3d pelvisP, Vector3d pelvisR, Vector3d leftAnkleP, Vector3d leftAnkleR, Vector3d rightAnkleP, Vector3d rightAnkleR);
        double* geometricIK(Vector3d p1, Matrix3d r1, Vector3d p7, Matrix3d r7, bool isLeft);

        Matrix3d Rroll(double phi);
        Matrix3d RPitch(double theta);
};