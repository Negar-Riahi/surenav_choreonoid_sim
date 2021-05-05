#include "headers/DCM.h"

DCMPlanner::DCMPlanner(double deltaZ, double stepTime, double doubleSupportTime, double dt, int stepCount, double alpha){
    this->deltaZ_ = deltaZ;
    this->tStep_ = stepTime;
    this->tDS_ = doubleSupportTime;
    if(alpha > 0.0 && alpha < 1.0)
        this->alpha_ = alpha;
    else
        throw "Invalid Value for alpha";
    this->dt_ = dt;
    this->stepCount_ = stepCount;

}

void DCMPlanner::setFoot(Vector3d rF[]){
    int length = sizeof(rF) / sizeof(rF[0]);
    rF_ = new Vector3d[length];
    rF_ = rF;
}

Vector3d* DCMPlanner::getXiTrajectory(Vector3d VRP[],Vector3d xiEOS[]){

    int length = 1/dt_ * tStep_ * stepCount_;
    Vector3d* xi_d = new Vector3d[length];
    int iter, stepNum;

    for (double time = 0 ; time < stepCount_ * tStep_ ; time += dt_){
        stepNum = fmod(time, tStep_);
        iter = time / tStep_;
        xi_d[iter] = VRP[iter] + exp(sqrt(K_G/deltaZ_) * (time = tStep_)) * (xiEOS[iter] - VRP[iter]);
    }

    Vector3d* xi_ds_i = new Vector3d[stepCount_];
    Vector3d* xi_ds_e = new Vector3d[stepCount_];

    for (int index = 0 ; index < stepCount_; index ++){
        if(index == 0){
            xi_ds_i[index] = xi_d[0];
            xi_ds_e[index] = VRP[index] + exp(sqrt(K_G/deltaZ_) * tDS_ * (1 - alpha_)) * (xi_d[0] - VRP[index]);
        }
        else{
            xi_ds_i[index] = VRP[index-1] + exp(-sqrt(K_G/deltaZ_) * tDS_ * alpha_) * (xi_d[index-1] - VRP[index-1]);
            xi_ds_e[index] = VRP[index] + exp(sqrt(K_G/deltaZ_) * tDS_ * (1 - alpha_)) * (xi_d[index-1] - VRP[index]);
        }
    }

    //double CDS_coefs[][];
    // calculate cubic trajectory coefs

    return xi_d;
}

Vector3d* DCMPlanner::getXiDot(){
    
}

Vector3d* DCMPlanner::updateVRP(Vector3d rF[]){
    Vector3d* VRP = new Vector3d[this->stepCount_];
    Vector3d deltaZ(0.0,0.0,deltaZ_);
    for (int i = 0 ; i < this->stepCount_; i ++)
        VRP[i] = rF[i] + deltaZ;
    return VRP;
}

Vector3d* DCMPlanner::addDS(){

}

Vector3d* DCMPlanner::updateXiEoS(Vector3d rVRP[]){

    Vector3d* xi_eos = new Vector3d[stepCount_];
    for (int i = stepCount_- 1 ; i >= 0; i--){
        if (i == stepCount_ - 1)
            xi_eos[i] = rVRP[i];
        else
            xi_eos[i] = rVRP[i + 1] + exp(-sqrt(K_G/deltaZ_) * tStep_) * (xi_eos[i+1] - rVRP[i+1]);
    }
    return xi_eos;
}