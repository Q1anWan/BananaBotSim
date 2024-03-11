#include <string>
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <string>
// All the webots classes are defined in the "webots" namespace
using namespace webots;
#define TIME_STEP 64
#define MAX_SPEED 6.28
// entry point of the controller
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  // initialize devices
  // initialize devices
  DistanceSensor *ps[8];
  
  std::string psNames[8] = {"ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"};

  for (int i = 0; i < 8; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i].c_str());
    ps[i]->enable(TIME_STEP);
  }

  Motor *leftMotor = robot->getMotor("left wheel motor");
  Motor *rightMotor = robot->getMotor("right wheel motor");
  leftMotor->setPosition(INFINITY);
  rightMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);
  rightMotor->setVelocity(0.0);
  // write actuators inputs
  double leftSpeed  = 0;
  double rightSpeed = 0;
  // feedback loop: step simulation until receiving an exit event
  while (robot->step(TIME_STEP) != -1) {
    // read sensors outputs
    double psValues[8];
    for (int i = 0; i < 8 ; i++)
      {psValues[i] = ps[i]->getValue();}
    // detect obstacles
    bool right_obstacle =
      psValues[0] > 80.0 ||
      psValues[1] > 80.0 ||
      psValues[2] > 80.0;
    bool left_obstacle =
      psValues[5] > 80.0 ||
      psValues[6] > 80.0 ||
      psValues[7] > 80.0;
    // process behavior

    if (left_obstacle) {
      // turn right
      leftSpeed  = 0.5 * MAX_SPEED;
      rightSpeed = -0.5 * MAX_SPEED;
    }
    else if (right_obstacle) { 
      // turn left
      leftSpeed  = -0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
    }
    else {
      // go straight
      leftSpeed  = 0.5 * MAX_SPEED;
      rightSpeed = 0.5 * MAX_SPEED;
    }
    // write actuators inputs
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
  }
  delete robot;
  return 0; //EXIT_SUCCESS
}