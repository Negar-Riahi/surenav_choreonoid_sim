#include <cnoid/SimpleController>
#include <cnoid/Sensor>
#include <cnoid/BodyLoader>

#include "headers/Robot.h"

using namespace std;
using namespace cnoid;

const double pgain[] = {35.0, 22.0, 0.0, 35.0, 22.0, 0.0};

const double dgain[] = {1.7,2.0,0.0,1.7,2.0,0.0};

class Surena: public SimpleController{

public:
    BodyPtr ioBody;
    double dt;

    AccelerationSensor* accelSensor;
    RateGyroSensor* gyro;

    Robot* surena;

    Vector3d dv;
    Vector3d attitude;

    virtual bool initialize(SimpleControllerIO* io) override{
        
        ioBody = io->body();
        dt = io->timeStep();

        surena = new Robot();

        accelSensor = ioBody->findDevice<AccelerationSensor>("WaistAccelSensor");
        io->enableInput(accelSensor);
        gyro = ioBody->findDevice<RateGyroSensor>("WaistGyro");
        io->enableInput(gyro);

        for(int i=0; i < 29; ++i){
            Link* joint = ioBody->joint(i);
            joint->setActuationMode(Link::JOINT_TORQUE);
            io->enableIO(joint);
        }
        
        //////////////////// Test !! Do not commit!!/////////////
        DCMPlanner temp(0.8,0.8,0.3,0.01,7);
        Vector3d* f = new Vector3d[6];
        f[0] << 0.0, 0.115, 0.0;
        f[1] << 0.0,-0.115,0.0;
        f[2] << 0.5,0.115,0.0;
        f[3] << 1.0,-0.115,0.0;
        f[4] << 1.5,0.115,0.0;
        f[5] << 2.0,-0.115,0.0;
        f[6] << 2.5, 0.115, 0.0;

        temp.setFoot(f);
        Vector3d* vrp = temp.updateVRP(f);
        //cout << vrp[4] << endl;
        Vector3d* xi_eos = temp.updateXiEoS(vrp);
        cout << xi_eos[0] << "\n----------\n" << xi_eos[1] << "\n----------\n"<< xi_eos[2] << "\n----------\n"<< xi_eos[3] << "\n----------\n" << xi_eos[4] << "\n----------\n" << xi_eos[5] << "\n----------\n" << xi_eos[6] << endl;
        ///////////////////////////////////////////////////////////
        return true;
    }

    virtual bool control() override{
        dv = accelSensor->dv();
        attitude = gyro->w();
        return true;
    }
private:

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(Surena)