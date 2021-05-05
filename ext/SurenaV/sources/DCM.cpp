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

Vector3d* DCMPlanner::getXiTrajectory(){

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
    xi_eos[stepCount_ - 1] = rVRP[stepCount_];
    for (int i = stepCount_-2 ; i > 0; i--){
        xi_eos[i] = rVRP[i + 1] + exp(-sqrt(K_G/deltaZ_) * tStep_) * (xi_eos[i+1] - rVRP[i+1]);
    }
    return xi_eos;
}