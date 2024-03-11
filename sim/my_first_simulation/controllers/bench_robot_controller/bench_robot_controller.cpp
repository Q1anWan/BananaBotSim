// File:          bench_robot_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Accelerometer.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#include "libpid-i-1.0.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace	PID;
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the RoFbot node
int main(int argc, char **argv) {
    // create the Robot instance.
    Robot *robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor *motor = robot->getMotor("motorname");
    //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
    //  ds->enable(timeStep);
    Motor *wheels[2];
    InertialUnit *ins;
    Accelerometer *accel;
    Gyro *gyro;
	//Run at 500Hz!!!
	PID_Pst_f pid_pst(0.5,0.0,0.1,0,0.005f,1,-1,1,-1,false,0,true,0.08);

    wheels[0] = robot->getMotor("motor_0");
    wheels[1] = robot->getMotor("motor_1");
    ins = robot->getInertialUnit("ins");
    accel = robot->getAccelerometer("accel");
    gyro = robot->getGyro("gyro");

	wheels[0]->enableTorqueFeedback(timeStep);
	wheels[1]->enableTorqueFeedback(timeStep);
	wheels[0]->setPosition(INFINITY);

	ins->enable(timeStep);
	accel->enable(timeStep);
	gyro->enable(timeStep);

    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1) {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();
		pid_pst.Calculate(ins->getRollPitchYaw()[1],0.001f*(float)timeStep);
		
        // Process sensor data here.

        // Enter here functions to send actuator commands, like:
        //  motor->setPosition(10.0);
		wheels[0]->setTorque(-pid_pst.Out());
		wheels[1]->setTorque(pid_pst.Out());
        // std::cout<<ins->getRollPitchYaw()[1]<<std::endl;
    };

    // Enter here exit cleanup code.

    delete robot;
    return 0;
}
