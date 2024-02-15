#include "main.h"
#include "pros/misc.h"

/*
 * helpful resources
 * path planner: https://path.jerryio.com/
 */

void redOffensive() {}
void redDefensive() {}

void blueOffensive() {}
void blueDefensive() {}
// ODOM-BASED PID MOVEMENT + BOOMERANG TEST
void redTest() {
  controller.clear();
  controller.print(0, 0, "odom PID + boomerang test");
  controller.rumble("....");
  chassis->setPose(0, 0, 0);
  // no boomerang
  controller.clear();
  controller.print(0, 0, "forwards 36 inches");
  pros::delay(0);
  controller.rumble(".");
  chassis->moveTo(0, 36, 0, 1e5);
  // turn
  controller.clear();
  controller.print(0, 0, "180, 2x 90 degree turns");
  pros::delay(0);
  controller.rumble(".");
  chassis->setPose(0, 0, 0);

  // 90 clockwise
  chassis->turnTo(1000, 0, 1e5);
  pros::delay(0);
  // 180
  chassis->turnTo(-1000, 0, 1e5);
  pros::delay(0);
  // 90
  chassis->turnTo(0, 1000, 1e5);

  // boomerang
  controller.clear();
  controller.print(0, 0, "forwards 24 inches, 90 turn");
  pros::delay(500);
  controller.rumble(".");
  chassis->setPose(0, 0, 0);
  chassis->moveTo(0, 24, 45, 1e5, false, true, 100);
}

// PATH FOLLOWING TEST
void blueTest() {}

// skills
void skills() {}