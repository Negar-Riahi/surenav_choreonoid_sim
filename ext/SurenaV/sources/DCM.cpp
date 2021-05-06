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
    if (stepCount > 3)
        this->stepCount_ = stepCount;
    else
        throw "at least 4 steps is needed for trajectory generator";

}

void DCMPlanner::setFoot(Vector3d rF[]){
    int length = sizeof(rF) / sizeof(rF[0]);
    rF_ = new Vector3d[length];
    rF_ = rF;
}

Vector3d* DCMPlanner::getXiTrajectory(){
    /*
        This function returns an array of positions with intervals of dt_
        It is best to call it at end of each step with new foot positions
    */
    this->updateVRP();
    this->updateXiEoS();
    this->updateSS();
    this->updateXiDSPositions();
    return xi_;
}

Vector3d* DCMPlanner::getXiDot(){
    int length = tStep_ * stepCount_ / dt_;
    Vector3d* xi_dot = new Vector3d[length];
    for (int i = 0 ; i < length ; i ++){
        int step = fmod((i * dt_), tStep_);
        xi_dot[i] = (xi_[i] - rVRP_[step]) * sqrt(K_G/deltaZ_);
    }
    return xi_dot;
}

Vector3d* DCMPlanner::getCoM(Vector3d COM_0){
    int length = tStep_ * stepCount_ / dt_;
    for (int i = 0; i < length; i++){
        Vector3d inte = Vector3d::Zero(3);
        for (int j = 0; j < i; j++)
            inte += dt_ * xi_[j] * exp(dt_/sqrt(deltaZ_/K_G));
        COM_[i] = (inte/sqrt(deltaZ_/K_G)) + COM_0 * exp(-i*dt_/sqrt(deltaZ_/K_G));
    }
    return COM_;
}

void DCMPlanner::updateVRP(){
    // Updates Virtual Repelant Points !! should be called after setFoot() !!
    Vector3d* VRP = new Vector3d[this->stepCount_];
    Vector3d deltaZ(0.0,0.0,deltaZ_);
    for (int i = 0 ; i < this->stepCount_; i ++)
        VRP[i] = rF_[i] + deltaZ;
}

void DCMPlanner::updateSS(){
    // Generates DCM trajectory without Double Support Phase
    int length = 1/dt_ * tStep_ * stepCount_;
    xi_ = new Vector3d[length];
    int iter, stepNum;

    for (double time = 0 ; time < stepCount_ * tStep_ ; time += dt_){
        stepNum = fmod(time, tStep_);
        iter = time / tStep_;
        xi_[iter] = rVRP_[iter] + exp(sqrt(K_G/deltaZ_) * (time = tStep_)) * (xiEOS_[iter] - rVRP_[iter]);
    }
}

void DCMPlanner::updateXiDSPositions(){
    /*
        This Function rounds single support trajectory bends
        for double support phase. 
        ! Double support starts and ends should be updated !
    */
    this->updateDS();
    Vector3d xi_dot_i, xi_dot_e;
    for(int step = 0 ; step < stepCount_; step ++){
        if (step == 0){
            xi_dot_i = (xiDSI_[step] - xi_[0]) * sqrt(K_G / deltaZ_);
            xi_dot_e = (xiDSE_[step] - rVRP_[0]) * sqrt(K_G / deltaZ_);
            Vector3d* coefs = this->minJerkInterpolate(xiDSI_[step],xiDSE_[step],xi_dot_i, xi_dot_e, tDS_);
            for (int i = 0; i < (1/dt_) * tDS_ * (1-alpha_); ++i){
                double time = dt_ * i;
                xi_[i] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3);
            }          
        }
        else{
            xi_dot_i = (xiDSI_[step] - rVRP_[step - 1]) * sqrt(K_G/deltaZ_);
            xi_dot_e = (xiDSE_[step] - rVRP_[step]) * sqrt(K_G/deltaZ_);
            Vector3d* coefs = this->minJerkInterpolate(xiDSI_[step],xiDSE_[step],xi_dot_i, xi_dot_e, tDS_);
            for (int i = (tStep_ * step)/dt_ - (tDS_ * alpha_ / dt_ ); i < (1/dt_) * tDS_ * -alpha_; ++i){
                double time = i * dt_;
                xi_[i] = coefs[0] + coefs[1] * time + coefs[2] * pow(time,2) + coefs[3] * pow(time,3);
            }
        }   
    }
}

void DCMPlanner::updateDS(){
    /*
        This function updates Double support start and end positions
    */
    xiDSI_ = new Vector3d[stepCount_];
    xiDSE_ = new Vector3d[stepCount_];

    for (int index = 0 ; index < stepCount_; index ++){
        if(index == 0){
            xiDSI_[index] = xi_[0];
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G/deltaZ_) * tDS_ * (1 - alpha_)) * (xi_[0] - rVRP_[index]);
        }
        else{
            xiDSI_[index] = rVRP_[index-1] + exp(-sqrt(K_G/deltaZ_) * tDS_ * alpha_) * (xi_[index-1] - rVRP_[index-1]);
            xiDSE_[index] = rVRP_[index] + exp(sqrt(K_G/deltaZ_) * tDS_ * (1 - alpha_)) * (xi_[index-1] - rVRP_[index]);
        }
    }
}

void DCMPlanner::updateXiEoS(){

    xiEOS_ = new Vector3d[stepCount_];
    for (int i = stepCount_- 1 ; i >= 0; i--){
        if (i == stepCount_ - 1)
            xiEOS_[i] = rVRP_[i];
        else
            xiEOS_[i] = rVRP_[i + 1] + exp(-sqrt(K_G/deltaZ_) * tStep_) * (xiEOS_[i+1] - rVRP_[i+1]);
    }
}

Vector3d* minJerkInterpolate(Vector3d theta_ini, Vector3d theta_f, Vector3d theta_dot_ini, Vector3d theta_dot_f, double tf){
    /* 
        Returns Cubic Polynomial with the Given Boundary Conditions
        https://www.tu-chemnitz.de/informatik//KI/edu/robotik/ws2016/lecture-tg%201.pdf
    */
    Vector3d* coefs = new Vector3d[4]; // a0, a1, a2, a3
    coefs[0] = theta_ini;
    coefs[1] = theta_dot_ini;
    coefs[2] = 3/pow(tf,2) * (theta_f - theta_ini) - 1/tf * (2 * theta_dot_ini + theta_dot_f);
    coefs[3] = -2/pow(tf,3) * (theta_f - theta_ini) + 1/pow(tf,2) * (theta_dot_ini + theta_dot_f);
    return coefs;
}