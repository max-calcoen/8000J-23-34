#include "robotFunctions.h"
#include "Graphy/Grapher.hpp"
#include "main.h"
#include "okapi/api.hpp"
#include "okapi/impl/control/iterative/iterativeControllerFactory.hpp"
#include "pros/misc.h"

void odomScreen() {
  // loop forever
  while (true) {
    lemlib::Pose pose =
        chassis->getPose(); // get the current position of the robot
    pros::lcd::print(0, "x: %f", pose.x);           // print the x position
    pros::lcd::print(1, "y: %f", pose.y);           // print the y position
    pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
    pros::delay(10);
  }
}

void flywheelScreen() {
  // create grapher
  std::shared_ptr<graphy::AsyncGrapher> grapher(
      new graphy::AsyncGrapher("Flywheel Velocity vs. Time"));

  // add data types
  grapher->addDataType("Desired Vel", COLOR_ORANGE);
  grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);

  // start grapher task
  grapher->startTask();

  while (true) {
    // update data
    grapher->update("Desired Vel", targetFlywheelSpeed);
    grapher->update("Actual Vel", currFlywheelSpeed);
    // run loop at 30ms intervals
    pros::delay(30);
  }
}

// https://www.desmos.com/calculator/sdhupmozc2
double filterJoystickInput(int input) {
  const int DEADZONE = 3;
  const double SCALE = 1.8;
  if (abs(input) < DEADZONE)
    return 0;
  return (input < 0) ? -1 : 1 * pow(abs(input), SCALE) / pow(127, SCALE - 1);
}

double flywheelSpeed = 0;
double currFlywheelSpeed = 0;

void flywheelTask() {
  // TODO: tune
  const double kP = 0.001;
  const double kI = 0.0001;
  const double kD = 0.0001;
  const int MOTOR_PORT = 1; // TODO: configure port

  okapi::IterativeVelPIDController fwController =
      okapi::IterativeControllerFactory::velPID(kP, kI, kD);

  // execute the movement
  fwController.setTarget(targetFlywheelSpeed);
  while (true) {
    fwController.setTarget(targetFlywheelSpeed);
    currFlywheelSpeed = flywheel->get_actual_velocity();
    fwController.controllerSet(fwController.step(currFlywheelSpeed));

    pros::delay(10); // run control loop at 10ms intervals
  }
}
void initFlywheelTask() { pros::Task fwTask(flywheelTask); }

void handleButtons() {
  // toggle flywhel
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y))
    targetFlywheelSpeed = targetFlywheelSpeed == 0 ? 600 : 0;
  // hold r1 for outtake
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    intake->move(-127);
  else
    intake->move(0);
  // hold r2 for intake (override outtake)
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    intake->move(127);
  else
    intake->move(0);
}