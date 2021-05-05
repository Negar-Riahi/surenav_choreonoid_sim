#include "headers/PID.h"

PID::PID(double kp, double ki, double kd, double timeStep){
    this->kp_ = kp;
    this->ki_ = ki;
    this->kd_ = kd;
    
    this->intI = 0.0;
    this->prevoiusError = 0.0;
    this->dt_ = timeStep;

}

double PID::getOutput(double desiredValue, double currentValue){
    /*
        computes next plant input
    */
    double error = currentValue - desiredValue;
    this->intI += error * dt_;
    double deriv = (error - this->prevoiusError) / dt_;
    this->prevoiusError = error;

    return kp_ * error + ki_ * this->intI + kd_ * deriv;
}