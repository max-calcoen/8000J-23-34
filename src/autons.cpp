#include "main.h"

/*
 * helpful resources
 * path planner: https://path.jerryio.com/
 */

void skills() { controller.print(0, 0, "skills!"); }

void redLeft() { controller.print(0, 0, "red left!"); }
void blueLeft() { controller.print(0, 0, "blue left!"); }
void redRight() { controller.print(0, 0, "red right!"); }
void blueRight() { controller.print(0, 0, "blue right!"); }

void redTest() {
  // testing odom-based pid movement
  controller.print(0, 0, "red test!");

  chassis->setPose(0, 0, 0);
  chassis->moveTo(0, 24, 15000,
                  600); // move to the point (10, 0) with a timeout
                        // of 1000 ms, and a maximum speed of 50
  chassis->setPose(0, 0, 0);
  chassis->turnTo(1, 1, 6000);
}
void blueTest() {
  // testing path following
  pros::lcd::print(3, "blue test!");
  chassis->setPose(0, 0, 0);
  chassis->moveTo(10, 0, 1000, 50); // move to the point (10, 0) with a timeout
                                    // of 1000 ms, and a maximum speed of 50
}