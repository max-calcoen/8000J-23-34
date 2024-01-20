#include "main.h"
#include "pros/misc.h"
ASSET(redleft1_txt);
ASSET(redleft2_txt);
ASSET(redleft3_txt);

ASSET(redright1_txt);
ASSET(redright2_txt);
ASSET(redright3_txt);

ASSET(blueleft1_txt)

ASSET(testpath_txt);
ASSET(skills1_txt);
ASSET(skills2_txt);
ASSET(skills3_txt);
ASSET(skills4_txt);

/*
 * helpful resources
 * path planner: https://path.jerryio.com/
 */

// https://cdn.discordapp.com/attachments/900591595315929098/1173631172799111199/IMG_4923.mov?ex=6564a834&is=65523334&hm=605aeb72f515a115db4f552baa34d7a58ad6319e95173bac4f3511c8e7496a97&
void redOffensive() {
  // init
  chassis->setPose(-52, 58, 135);

  // hit matchload near goal with wings
  wings.set_value(true);
  pros::delay(500);
  wings.set_value(false);
  pros::delay(200);

  // collect middle ball
  intake->move(127);
  chassis->turnTo(-6.5, 1, 100);
  chassis->moveTo(-6.5, 1, 135, 12000);
  // wait to collect
  pros::delay(500);
  // turn to goal
  chassis->turnTo(-37, 0, 1e5);
  // push balls in
  chassis->moveTo(-37, 0, -90, 1500, true);
  // halfway through, outtake and trigger wings
  chassis->waitUntilDist(3);
  intake->move(-127);
  wings.set_value(true);
  // wait till movement done
  chassis->waitUntilDist(1e5);

  // move towards next
  chassis->follow(redleft1_txt, 1e5, 15.0, false, false);
  wings.set_value(false);
  chassis->turnTo(0, 24, 1e5);
  intake->move(127);
  // collect
  chassis->moveTo(0, 24, 60, 1500, false);
  pros::delay(100);
  // reset on barrier
  chassis->setPose(-8, 24, 60);

  // drop off ball to collect other one
  chassis->turnTo(-36, 36, 1e5);
  // no boomerang
  chassis->moveTo(-36, 36, -60, 1e5, true);
  // release ball (COMMENTED OUT)
  chassis->waitUntilDist(15);
  // intake->move(-127);
  // wait until reach
  chassis->waitUntilDist(1e5);
  // sweep balls
  chassis->setPose(-36, 36, -60);
  chassis->follow(redleft2_txt, 1.5e3, 10, true);
  chassis->waitUntilDist(6);
  // outtake to push under
  intake->move(-127);
  chassis->waitUntilDist(1e5);
  // back up just in case
  chassis->moveTo(-60, 53, 180, 1e5, false, false, 0, 0.6, 30);
}
void redDefensive() {
  // init
  intake->move(127);
  chassis->setPose(-36, -54, 0);
  chassis->moveTo(-36, -7, 0, 1e5);
  // push over
  chassis->turnTo(3, -7, 1e5);
  wings.set_value(true);
  chassis->moveTo(3, -7, 90, 1000);
  chassis->follow(redright2_txt, 1e5, 15, true, false);
  chassis->waitUntilDist(2);
  wings.set_value(false);
  chassis->waitUntilDist(1e5);
  chassis->turnTo(-45, -13, 1e5);
  chassis->moveTo(-45, -13, -90, 500, true, true, 0, 0.6, 60);
  chassis->waitUntilDist(2);
  intake->move(-127);
  chassis->waitUntilDist(1e5);
  chassis->follow(redright3_txt, 1e5, 15, false, false);
  intake->move(-127);
  chassis->moveTo(-18, -57, 90, 1e5, false, true, 0, 0, 100);
}

void blueOffensive() { controller.print(0, 0, "blue right!"); }
void blueDefensive() {
  // init
  intake->move(127);
  wings.set_value(true);
  pros::delay(500);
  chassis->setPose(36, 53, 0);
  chassis->turnTo(37, 70, 1e5, false, true);
  pros::delay(500);
  wings.set_value(false);
}
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
void blueTest() {
  controller.print(0, 0, "path following test");

  chassis->follow(testpath_txt, 1e5, 10, false);
  controller.rumble(".");
}

// skills
void skills() {
  // init
  chassis->setPose(48, 56, 242);
  // back up a tiny bit
  chassis->moveTo(51.226, 57.715, 242, 150, true, false, 0, 0.6, 30);
  flywheel->move(-127);
  // wait while shooting
  pros::delay(40e3);
  flywheel->move(0);
  // go to other side
  intake->move(-127);
  chassis->follow(skills1_txt, 1e5, 10);
  // push under goal
  intake->move(-127);
  chassis->follow(skills2_txt, 1e3, 10, true);
  chassis->waitUntilDist(5);
  wings.set_value(true);
  chassis->waitUntilDist(10);
  wings.set_value(false);
  chassis->waitUntilDist(1e5);
  // back up from goal
  chassis->moveTo(-60, 32, 180, 1e5, false, false);
  // swing around
  chassis->turnTo(-24, 32, 2e3);
  chassis->moveTo(-24, 32, 90, 1e5);
  chassis->follow(skills3_txt, 5e3, 10, true);
  chassis->waitUntilDist(5);
  wings.set_value(true);
  chassis->waitUntilDist(1e5);
  // back up
  chassis->moveTo(-10, 8, 180, 2e3, false, false);
  wings.set_value(false);
  // swing again
  chassis->turnTo(-8, -8, 1e5);
  chassis->follow(skills4_txt, 2e3, 10, true);
  chassis->waitUntilDist(3);
  wings.set_value(true);
  chassis->waitUntilDist(1e5);
  // back up just in case
  chassis->moveTo(0, -16, 270, 1e5, false, false);
}