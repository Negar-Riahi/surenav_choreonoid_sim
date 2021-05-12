#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>

#include "headers/Robot.h"

using namespace std;
using namespace cnoid;

const double pgain[] = {
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0, 8000.0, 8000.0, 8000.0,
    3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 3000.0, 
    8000.0, 8000.0, 8000.0 };
    
const double dgain[] = {
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0,
    100.0, 100.0, 100.0 };

class Surena: public SimpleController{

public:
    BodyPtr ioBody;
    double dt;

    vector<double> qref;
    vector<double> qold;

    Robot* surena;

    virtual bool initialize(SimpleControllerIO* io) override{
        
        ioBody = io->body();
        dt = io->timeStep();

        surena = new Robot;

        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(accelSensor);
        gyro = ioBody->findDevice<RateGyroSensor>("WaistGyro");
        io->enableInput(gyro);

        for(int i=0; i < 29; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
            qref.push_back(joint->q());
        }
        qold = qref;

        simTime_ = 0.0;
        simIterator_ = 0;

        return true;
    }

    virtual bool control() override{
        dv = accelSensor->dv();
        attitude = gyro->w();

        // Set Motor Angles
        qref = surena->spinOffline();
        //cout << qref[0] << " " << qref[1] << " " << qref[2] << " " << qref[3] << " " << qref[4] << " " << qref[5] << " " << endl;
        //cout << qref[13] << " " << qref[14] << " " << qref[15] << " " << qref[16] << " " << qref[17] << " " << qref[18] << " " << endl;
        // Send Motor Commands to Body
        for(int i=0; i < 29; ++i){
            Link* joint = ioBody->joint(i);
            double q = joint->q();
            double dq = (q - qold[i]) / dt;
            double u = (qref[i] - q) * pgain[i] + (0.0 - dq) * dgain[i];
            qold[i] = q;
            joint->u() = u;
        }
        simIterator_ ++;
        simTime_ += dt;
        return true;
    }
private:

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;

    Vector3d dv;
    Vector3d attitude;
    double simTime_;
    int simIterator_;
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Surena)