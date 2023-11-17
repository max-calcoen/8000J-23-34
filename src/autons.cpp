#include "main.h"
ASSET(testpath_txt);
ASSET(path2_txt);

/*
 * helpful resources
 * path planner: https://path.jerryio.com/
 */

void skills() { controller.print(0, 0, "skills!"); }

void redLeft() { controller.print(0, 0, "red left!"); }
void blueLeft() { controller.print(0, 0, "blue left!"); }
void redRight() { controller.print(0, 0, "red right!"); }
void blueRight() { controller.print(0, 0, "blue right!"); }

// ODOM-BASED PID MOVEMENT + BOOMERANG TEST
void redTest() {
  controller.clear();
  controller.print(0, 0, "odom PID + boomerang test");
  controller.rumble("..");
  pros::delay(500);
  chassis->setPose(0, 0, 0);
  // no boomerang
  controller.clear();
  controller.print(0, 0, "forwards 24 inches");
  pros::delay(500);
  controller.rumble(".");
  chassis->moveTo(0, 24, 0, 1e5);

  // turn
  controller.clear();
  controller.print(0, 0, "180, 2x 90 degree turns");
  pros::delay(500);
  controller.rumble(".");
  chassis->setPose(0, 0, 0);

  // 90 counterclockwise, assuming theta 0 is unit circle 0
  chassis->turnTo(0, 1, 1e5);
  pros::delay(500);
  // 180
  chassis->turnTo(0, -1, 1e5);
  pros::delay(500);
  // 90
  chassis->turnTo(1, 0, 1e5);

  // boomerang
  controller.clear();
  controller.print(0, 0, "forwards 24 inches, 90 turn");
  pros::delay(500);
  controller.rumble(".");
  chassis->setPose(0, 0, 0);
  chassis->moveTo(100, 100, 90, 1e5);
}

// PATH FOLLOWING TEST
void blueTest() {
  controller.print(0, 0, "path following test");

  chassis->follow(testpath_txt, 1e5, 15, false);
  controller.rumble(".");
  pros::delay(1000);
  // follow the same path backwards
  chassis->follow(testpath_txt, 1e5, 15, false, false);
}