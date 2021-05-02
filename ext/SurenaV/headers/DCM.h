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

class DCMPlanner{
    public:
        DCMPlanner(double deltaZ, double stepTime, double doubleSupportTime, double alpha, double dt, int stepCount);
        void setFoot(Vector3d rF[]);
        Vector3d* getXiTrajectory();
        Vector3d* getXiDot();

    private:
        // Design Parameters
        double deltaZ_;
        double tStep_;
        double tDS_;
        double alpha_;

        // Trajectory Arrays
        Vector3d* xi_;
        Vector3d* xiDot_;
        Vector3d* COM_;
        Vector3d* ZMP_;

        // Functions for generating trajectories
        Vector3d* updateVRP(Vector3d rF[], int count);
        Vector3d* addDS();
        Vector3d* updateXi();
};