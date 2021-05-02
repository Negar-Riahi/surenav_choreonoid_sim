#pragma once

#include "iostream"
#include "fstream"
#include <vector>
#include "math.h"
#include "cmath"

class PID{
    public:
        PID(double kp, double ki, double kd);

    private:
        double kp_;
        double ki_;
        double kd_;

        
};