#include "main.h"
ASSET(redleft1_txt);
ASSET(redleft2_txt);
ASSET(redleft3_txt);
ASSET(redleft4_txt);
ASSET(testpath_txt);

/*
 * helpful resources
 * path planner: https://path.jerryio.com/
 */

void skills() { controller.print(0, 0, "skills!"); }

// https://cdn.discordapp.com/attachments/900591595315929098/1173631172799111199/IMG_4923.mov?ex=6564a834&is=65523334&hm=605aeb72f515a115db4f552baa34d7a58ad6319e95173bac4f3511c8e7496a97&
void redLeft() {
  controller.rumble(".");
  // init
  chassis->setPose(-51, 57, 135);

  // hit matchload near goal with wings
  wings.set_value(true);
  pros::delay(500);
  wings.set_value(false);
  pros::delay(200);

  // collect middle ball
  intake->move(127);
  chassis->turnTo(-8, 8, 100);
  chassis->moveTo(-7, 7, 135, 1e5);
  pros::delay(500);
  controller.print(0, 0, "1");
  // turn to goal
  chassis->turnTo(-35, 7, 1e5);
  // push balls in
  chassis->moveTo(-35, 7, -90, 1e5, true);
  // halfway through, outtake and trigger wings
  chassis->waitUntilDist(5);
  intake->move(-127);
  wings.set_value(true);
  // wait till movement done
  chassis->waitUntilDist(1e5);
  // move towards next
  chassis->follow(redleft1_txt, 1e5, 15.0, false, false);
  wings.set_value(false);
  chassis->turnTo(0, 23, 1e5);
  intake->move(127);
  chassis->moveTo(3, 24, 240, 1e5, false);

  // drop off ball to collect other one
  chassis->turnTo(-35, 39, 1e5);
  // no boomerang
  chassis->moveTo(-35, 39, 0, 1e5, true, true, 0, 0);
  // release ball
  chassis->waitUntilDist(15);
  intake->move(-127);
  // wait until reach
  chassis->waitUntilDist(1e5);
  intake->move(0);
  // follow path to get other ball
  chassis->follow(redleft2_txt, 1e5, 15, true);
  chassis->waitUntilDist(20);
  intake->move(127);
  chassis->waitUntilDist(1e5);
  // wait for ball to enter intake
  pros::delay(300);
  // back up to other balls
  chassis->moveTo(-40, 60, 90, 1e5, false, false);
  // spin around
  chassis->turnTo(-60, 60, 1e5);
  // get wings out to poke ball from matchload zone
  wings.set_value(true);
  chassis->follow(redleft3_txt, 1e5, 10, true);
  // halfway through movement put wings away
  chassis->waitUntilDist(15);
  wings.set_value(false);
  // push under again
  chassis->follow(redleft4_txt, 1e5, 10, false);
  // get wings out to poke ball from matchload zone
  wings.set_value(true);
  chassis->follow(redleft3_txt, 1e5, 10, true);
  // halfway through movement put wings away
  chassis->waitUntilDist(15);
  wings.set_value(false);
}
void blueLeft() { controller.print(0, 0, "blue left!"); }
void redRight() { controller.print(0, 0, "red right!"); }
void blueRight() { controller.print(0, 0, "blue right!"); }

// ODOM-BASED PID MOVEMENT + BOOMERANG TEST
void redTest() {
  controller.clear();
  controller.print(0, 0, "odom PID + boomerang test");
  controller.rumble("..");
  chassis->setPose(0, 0, 0);
  // no boomerang
  // controller.clear();
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