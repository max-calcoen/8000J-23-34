#include "main.h"

void skills() {}

void redLeft() {}
void blueLeft() {}
void redRight() {}
void blueRight() {}

void redTest() {
  pros::lcd::print(3, "red test!");
  /*
    // file name: path.txt
    // timeout: 2000 ms
    // lookahead distance: 15 inches
    chassis.follow("path.txt", 2000, 15);
    // follow the next path, but with the robot going backwards
    chassis.follow("path2.txt", 2000, 15, true);
*/
}
void blueTest() {
  pros::lcd::print(3, "blue test!");
  chassis->setPose(0, 0, 0);
  chassis->moveTo(10, 0, 1000, 50); // move to the point (10, 0) with a timeout
                                    // of 1000 ms, and a maximum speed of 50
}