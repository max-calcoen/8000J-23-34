#include "robotFunctions.h"
#include "main.h"
#include "pros/misc.h"

bool frontWingsOn = false;
bool backLeftWingOn = false;
bool backRightWingOn = false;
bool kickerStatus = false;
bool hangUp = false;
// TODO: config ports
pros::ADIDigitalOut front_wings('A');
pros::ADIDigitalOut back_wing_left('G');
pros::ADIDigitalOut back_wing_right('F');
pros::ADIDigitalOut hang('H');

void odomScreen() {
  // loop forever
  while (true) {
    lemlib::Pose currPose =
        skillsChassis->getPose(); // get the current position of the robot
    pros::lcd::print(0, "x: %f", currPose.x);           // print the x position
    pros::lcd::print(1, "y: %f", currPose.y);           // print the y position
    pros::lcd::print(2, "heading: %f", currPose.theta); // print the heading
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
    back_wing_right.set_value(backRightWingOn);
  }
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    if (backRightWingOn && backLeftWingOn) {
      back_wing_left.set_value(false);
      back_wing_right.set_value(false);
      backRightWingOn = false;
      backLeftWingOn = false;
    } else {
      back_wing_left.set_value(true);
      back_wing_right.set_value(true);
      backRightWingOn = true;
      backLeftWingOn = true;
    }
  }
  // toggle hang
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
    hangUp = !hangUp;
    hang.set_value(hangUp);
  }
  // hold kicker
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
    const int SPEED = 127;
    if (kickerStatus)
      kicker->move(0);
    else
      kicker->move(SPEED);
    kickerStatus = !kickerStatus;
  }
}