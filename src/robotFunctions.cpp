#include "robotFunctions.h"
#include "Graphy/Grapher.hpp"
#include "main.h"

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
  // TODO: implement these variables
  double TARGET_VEL = 0;
  double CURRENT_VEL = 0;
  // Create grapher
  std::shared_ptr<graphy::AsyncGrapher> grapher(
      new graphy::AsyncGrapher("Flywheel Velocity vs. Time"));

  // Add data types
  grapher->addDataType("Desired Vel", COLOR_ORANGE);
  grapher->addDataType("Actual Vel", COLOR_AQUAMARINE);

  // Start grapher task
  grapher->startTask();

  while (true) {
    // Update data
    grapher->update("Desired Vel", TARGET_VEL);
    grapher->update("Actual Vel", CURRENT_VEL);

    pros::delay(10);
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