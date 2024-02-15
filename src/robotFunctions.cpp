#include "robotFunctions.h"
#include "main.h"
#include "pros/misc.h"

bool frontWingsOn = false;
bool backLeftWingOn = false;
bool backRightWingOn = false;
bool hangUp = false;
// TODO: config ports
pros::ADIDigitalOut front_wings('A');
pros::ADIDigitalOut hang('H');
pros::ADIDigitalOut back_wing_left('G');
pros::ADIDigitalOut back_wing_right('F');

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

// https://www.desmos.com/calculator/sdhupmozc2
double filterJoystickInput(int input, double scale) {
  const int DEADZONE = 3;
  if (std::abs(input) < DEADZONE)
    return 0;
  return ((input < 0) ? -1 : 1) * pow(std::abs(input), scale) /
         pow(127, scale - 1);
}

void handleButtons() {
  // hold r1 for outtake
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    intake = -127;
  else
    intake = 0;
  // hold r2 for intake (override outtake)
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    intake = 127;

  // toggle wings
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
    frontWingsOn = !frontWingsOn;
    front_wings.set_value(frontWingsOn);
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
    backLeftWingOn = !backLeftWingOn;
    back_wing_left.set_value(backLeftWingOn);
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    backRightWingOn = !backRightWingOn;
    back_wing_left.set_value(backRightWingOn);
  }
  // toggle hang
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    hangUp = !hangUp;
    hang.set_value(hangUp);
  }
}