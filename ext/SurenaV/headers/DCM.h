#pragma once

#include"Eigen/Dense"
#include "Eigen/eiquadprog.h"
#include "Eigen/Core"
#include "Eigen/Cholesky"
#include "Eigen/LU"
#include <Eigen/Geometry>

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

using namespace Eigen;
using namespace std;


const double K_G = 9.81;

class DCMPlanner{
    friend class Surena;
    public:
        DCMPlanner(double deltaZ, double stepTime, double doubleSupportTime, double dt, int stepCount = 6, double alpha = 0.5);
        void setFoot(Vector3d rF[]);
        Vector3d* getXiTrajectory();
        Vector3d* getXiDot();

    private:
        // Design Parameters
        double deltaZ_;
        double tStep_;
        double tDS_;
        double alpha_;

        double dt_;         // sampling time
        int stepCount_;     // trajectories will be generated over how many steps

        // Trajectory Arrays
        Vector3d* xi_;
        Vector3d* xiDot_;
        Vector3d* COM_;
        Vector3d* ZMP_;

        Vector3d* rF_;
        //Vector3d* rVRP_;

        // Functions for generating trajectories
        Vector3d* updateVRP(Vector3d rF[]);
        Vector3d* addDS();
        Vector3d* updateXiEoS(Vector3d rVRP[]);
};