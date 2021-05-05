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
        // Controller Gains 
        double kp_;
        double ki_;
        double kd_;

        double prevoiusError_;   // Controller Previous Error
        double intI_;            // Controller Integrator

        double getOutput(double deiredValue, double currentValue);
};
