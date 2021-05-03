#pragma once

#include "headers/DCM.h"
#include "headers/Link.h"
#include "headers/PID.h"

using namespace std;

class Robot{
    public:
        Robot();

        void createLinkTree();

        vector<double> spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time);
        vector<double> spinOffline();

    private:

        DCMPlanner* trajectoryPlanner_;

        _Link* pelvis_;
        _Link* rightHipRoll_;
        _Link* rightHipPitch_;
        _Link* rightHipYaw_;
        _Link* rightKnee_;
        _Link* rightAnklePitch_;
        _Link* rightAnkleRoll_;

        _Link* lefttHipRoll_;
        _Link* leftHipPitch_;
        _Link* leftHipYaw_;
        _Link* leftKnee_;
        _Link* leftAnklePitch_;
        _Link* leftAnkleRoll_;

        PID* DCMController_;
        PID* CoMController_;
};