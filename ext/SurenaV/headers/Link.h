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


class _Link{
    public:
        _Link(short int ID, short int parentID, short int numChilds, short int childID);
        short int getID();
        double q();
        short int getChildID();
        short int getParentID();

    private:
        short int ID_;
        Vector3d worldAttitude; // Rotation relative to world frame
        Vector3d worldPose;     // Pose Relative to world frame
        double q_;               // Link config (theta in DH)
        short int childID_;
        short int parentID_;
};