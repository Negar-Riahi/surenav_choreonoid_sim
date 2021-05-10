#include "headers/Robot.h" 

Robot::Robot(){
    short int pelvis_children[3] = {0, 13, 26};
    _Link pelvis(30,"pelvis",30,3,pelvis_children,0.0);

    ///////////// Right Leg Parameters ////////////////////
    short int rHipR_children[1] = {1};      // RHipP
    _Link rHipR(0,"rHipR",30,1,rHipR_children,0.09);
    joints_.push_back(rHipR);
    short int rHipP_children[1] = {2};      // RHipY
    _Link rHipP(1,"rHipP",0,1,rHipP_children,0.0);
    joints_.push_back(rHipP);
    short int rHipY_children[1] = {3};      // RKnee
    _Link rHipY(2,"rHipY",1,1,rHipY_children,0.0);
    joints_.push_back(rHipY);
    short int rKnee_children[1] = {4};      // RAnkleP
    _Link rKnee(3,"rKnee",2,1,rKnee_children,0.3535);
    joints_.push_back(rKnee);
    short int rAnkleP_children[1] = {5};    // RAnkleR
    _Link rAnkleP(4,"rAnkleP",3,1,rAnkleP_children,0.3);
    joints_.push_back(rAnkleP);
    _Link rAnkleR(1,"rAnkleR",4,0,nullptr,0.0);
    joints_.push_back(rAnkleR);

    ///////////// Left Leg Parameters ////////////////////
    short int lHipR_children[1] = {1};      // LHipP
    _Link lHipR(13,"lHipR",30,1,rHipR_children,0.09);
    joints_.push_back(lHipR);
    short int lHipP_children[1] = {2};      // LHipY
    _Link lHipP(14,"lHipP",13,1,rHipP_children,0.0);
    joints_.push_back(lHipP);
    short int lHipY_children[1] = {3};      // LKnee
    _Link lHipY(15,"lHipY",14,1,rHipY_children,0.0);
    joints_.push_back(lHipY);
    short int lKnee_children[1] = {4};      // LAnkleP
    _Link lKnee(16,"lKnee",15,1,rKnee_children,0.3535);
    joints_.push_back(lKnee);
    short int lAnkleP_children[1] = {5};    // LAnkleR
    _Link lAnkleP(17,"lAnkleP",16,1,rAnkleP_children,0.3);
    joints_.push_back(lAnkleP);
    _Link lAnkleR(18,"lAnkleR",17,0,nullptr,0.0);
    joints_.push_back(lAnkleR);

    joints_.push_back(pelvis);

    trajectoryPlanner_ = new DCMPlanner(0.6, 1.0, 0.3, 0.01, 6, 0.5);
}

vector<double> Robot::spinOnline(VectorXd forceSensor, Vector3d gyro, Vector3d accelerometer, double time){
    // TODO
    // Add CoM Estimation
    // Add DCM + CoM controllers
}

vector<double> Robot::spinOffline(){
    Vector3d* f = new Vector3d[6];
    
    f[0] << 0.0, -0.09, 0.0;
    f[1] << 0.4, 0.09, 0.0;
    f[2] << 0.8, -0.09, 0.0;
    f[3] << 1.2, 0.09, 0.0;
    f[4] << 1.6, -0.09, 0.0;
    f[5] << 2.0, 0.09, 0.0;

    trajectoryPlanner_->setFoot(f);
    trajectoryPlanner_->getXiTrajectory();
    trajectoryPlanner_->getXiDot();
    Vector3d com(0.0,0.0,0.8);
    trajectoryPlanner_->getCoM(com);

    // TODO:
    // Get Ankle Trajectory and call doIK
    // Hope it goes well in simulation
}

void Robot::doIK(Vector3d pelvisP, Vector3d pelvisR, Vector3d leftAnkleP, Vector3d leftAnkleR, Vector3d rightAnkleP, Vector3d rightAnkleR){
    // Calculates and sets Robot Leg Configuration at each time step
    double* q_left = this->geometricIK(pelvisP, pelvisR, leftAnkleP, leftAnkleR, true);
    double* q_right = this->geometricIK(pelvisP, pelvisR, rightAnkleP, rightAnkleR, false);
    
    for(int i = 0; i < 6; i ++){

    }
}

Matrix3d Robot::Rroll(double phi){
    // helper Function for Geometric IK
    MatrixXd R(3,3);
    double c=cos(phi);
    double s=sin(phi);
    R<<1,0,0,0,c,-1*s,0,s,c;
    return R;
}

Matrix3d Robot::RPitch(double theta){
    // helper Function for Geometric IK
    MatrixXd Ry(3,3);
    double c=cos(theta);
    double s=sin(theta);
    Ry<<c,0,s,0,1,0,-1*s,0,c;
    return Ry;

}

double* Robot::geometricIK(Vector3d p1, Matrix3d r1, Vector3d p7, Matrix3d r7, bool isLeft){
    /*
        Geometric Inverse Kinematic for Robot Leg (Section 2.5  Page 53)
        Reference: Introduction to Humanoid Robotics by Kajita        https://www.springer.com/gp/book/9783642545351
    */
   double a = joints_[3].length_;
   double b = joints_[4].length_;
   double* q = new double[6];
   Vector3d D;
   if (isLeft)
        D << 0.0,-joints_[0].length_,0.0;
    else
        D << 0.0,joints_[0].length_,0.0;
   Vector3d r = r7.transpose() * (p1 + r1 * D - p7);
   double C = r.norm();
   double c5 = (pow(C,2) - pow(a,2) - pow(b,2)/(2 * a * b));
   if (c5 >= 1)
        q[3] = 0.0;
    else if(c5 <= -1)
        q[3] = M_PI;
    else
        q[3] = acos(c5);       // Knee Pitch
    double q6a = asin((a/C) * sin(M_PI - q[3]));
    q[5] = atan2(r(1),r(2));   //Ankle Roll
    if (q[5] > M_PI/2) 
        q[5] = q[5] - M_PI;
    else if (q[5] < -1*M_PI / 2)
        q[5] = q[5] + M_PI;
    int sign_r2 = 1;
    if(r(2) < 0)
        sign_r2 = -1;
    q[4] = -atan2(r(0),sign_r2 * sqrt(pow(r(1),2) + pow(r(2),2))) - q6a;      // Ankle Pitch
    Matrix3d R = r1.transpose() * r7 * (Rroll(-q[5]),RPitch(-q[3] - q[4]));
    q[0] = atan2(-R(0,1),R(1,1));         // Hip Yaw
    q[1] = atan2(R(2,1), -R(0,1) * sin(q[0]) + R(1,1) * cos(q[0]));           // Hip Roll
    q[2] = atan2(-R(2,0), R(2,2));        // Hip Pitch
    return q;
}