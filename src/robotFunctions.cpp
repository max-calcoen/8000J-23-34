#include "robotFunctions.h"
#include "main.h"
#include "pros/misc.h"

bool flywheelOn = false;
bool wingsOn = false;
bool hangUp = false;

pros::ADIDigitalOut wings('a');
pros::ADIDigitalOut hang('h');

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
  pros::lcd::print(0, "flywheel: %f", flywheel->get_actual_velocity());
}

// https://www.desmos.com/calculator/sdhupmozc2
double filterJoystickInput(int input, double scale) {
  const int DEADZONE = 3;
  if (std::abs(input) < DEADZONE)
    return 0;
  return ((input < 0) ? -1 : 1) * pow(std::abs(input), scale) /
         pow(127, scale - 1);
}

double flywheelSpeed = 0;
double currFlywheelSpeed = 0;

void handleButtons() {
  // toggle flywhel
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
    if (flywheelOn) {
      flywheel->move(0);
    } else {
      // MAX SPEED
      flywheel->move(127);
    }
    flywheelOn = !flywheelOn;
  }
  // hold r1 for outtake
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    intake->move(-127);
  else
    intake->move(0);
  // hold r2 for intake (override outtake)
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    intake->move(127);

  // toggle wings
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    wingsOn = !wingsOn;
    wings.set_value(wingsOn);
  }
  // toggle hang
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    hangUp = !hangUp;
    hang.set_value(hangUp);
  }
}