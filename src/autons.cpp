#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/misc.h"
#include "robotFunctions.h"

#include <iomanip>
#include <sstream> // For std::ostringstream to convert numbers to strings.

/*
 * helpful resources
 * path planner: https://path.jerryio.com/
 */
void nearWP() {
  bool elims = false;
  if (elims) {
    lemlib::Chassis &c = *matchChassis;
    // init
    c.setPose(-32, -54, 0);
    intake = 127;

    // push preload ball with wings near goal
    front_wings.set_value(true);
    frontWingsOn = true;
    pros::delay(200);
    front_wings.set_value(false);
    frontWingsOn = false;
    // get middle ball
    c.moveTo(-24, -8, 0, 2500);
    // outake and push balls over
    // move back and align
    c.moveTo(-40, -10, 90, 2500, false, false);
    intake = -127;
    front_wings.set_value(true);
    frontWingsOn = true;
    c.moveTo(-8, -10, 90, 2500);

    c.turnTo(0, -2, 1500);
    front_wings.set_value(false);
    frontWingsOn = false;
    c.moveTo(-48, -48, 45, 1e5);
    c.turnTo(-36, -60, 1500);
    // push preload into goal
    c.moveTo(-67, -30, 180, 2500, false, false);
    intake = -127;
    c.moveTo(-20, -60, 90, 1e5);
  } else {
    lemlib::Chassis &c = *matchChassis;
    // init
    c.setPose(52.5, 55, 315);
    intake = -127;
    // move back to push preloads in
    c.moveTo(66, 30, 0, 2300, false, false, 30);
    // move to get ball out of corner
    c.moveTo(36, 53, 270, 1e5, true, true, 0, 0.8);
    c.waitUntilDist(12);
    back_wing_right.set_value(true);
    c.waitUntilDist(30);
    back_wing_right.set_value(false);
    c.waitUntilDist(1e5);
    c.moveTo(66, 30, 0, 1e5, false, false, 30, 0.7);
    c.moveTo(13, 60, 270, 1e5, false, true, 0, 0.6);
  }
}
void nearMidrushCenter() {}
void nearMidrushBarrier() {}

void farSixBall() {
  // MATCH CHASSIS
  lemlib::Chassis &c = *matchChassis;
  // init
  c.setPose(-12, 60, 90);
  intake = 127;
  // move to pick up ball under center bar
  c.moveTo(-10, 60, 90, 300);
  // wait for ball to intake
  pros::delay(100);
  // back up and get ball out of corner
  // motion chaining
  c.moveTo(-36, 60, 90, 300, false, false);
  c.moveTo(-60, 30, 0, 3800, true, false, 10, 0.4);
  // c.waitUntilDist(35);
  // back_wing_left.set_value(true);
  // c.waitUntilDist(43);
  // back_wing_left.set_value(false);
  c.waitUntilDist(1e5);
  // move out from goal
  c.moveTo(-56, 56, 25, 1e5, false, true, 0, 0);
  // turn to face goal again
  c.turnTo(-60, 30, 1e5);
  // push num 2
  c.moveTo(-60, 24, 180, 4000, true, true, 0, 0.4);
  c.waitUntilDist(10);
  intake = -127;
  c.waitUntilDist(1e5);
  // reset on goal barrier
  c.setPose(-60, 33, c.getPose().theta);
  // move out
  c.moveTo(-48, 48, 20, 1e5, false, false);
  // turn to next ball
  c.turnTo(-6, 24, 1e5);
  // go to pick it up
  intake = 127;
  c.moveTo(-10, 36, 115, 1e5);
  // drop it off
  c.turnTo(-15, 24, 1e5);
  c.moveTo(-15, 24, c.getPose().theta, 1e5, true);
  c.waitUntilDist(5);
  intake = -127;
  c.waitUntilDist(1e5);
  // let ball roll out
  pros::delay(500);
  // get next ball
  c.turnTo(24, 0, 1e5);
  intake = 127;
  c.moveTo(24, 0, c.getPose().theta, 1e5, false, true, 0, 0);
  c.turnTo(-44, 0, 1e5);
  // move back to push balls in
  back_wing_left.set_value(true);
  back_wing_right.set_value(true);
  c.moveTo(-48, 0, 90, 2500, false, false);
  c.setPose(-44, 0, c.getPose().theta);

  // turn around and put other ball in
  c.moveTo(-36, 0, c.getPose().theta, 1e5);
  c.turnTo(-44, 0, 1e5);
  intake = -127;
  front_wings.set_value(true);
  frontWingsOn = true;
  c.moveTo(-44, 0, 270, 1e5);
}
void farMidrushCenter() {}
void farMidrushBarrier() {}

// ODOM-BASED PID MOVEMENT + BOOMERANG TEST
void test() {
  // test bools
  bool testFwd = false;
  bool testBwd = false;
  bool testTurn = false;
  bool testBoomerang = true;
  bool testMotionChaining = false;
  // MATCH CHASSIS
  lemlib::Chassis &c = *matchChassis;

  if (testFwd) {
    c.setPose(0, 0, 0);
    // no boomerang
    controller.clear();
    controller.print(0, 0, "forwards 24 inches");
    c.moveTo(0, 36, 0, 1e5);
  }
  if (testBwd) {
    c.setPose(0, 0, 0);
    // no boomerang
    controller.clear();
    controller.print(0, 0, "backwards 24 inches");
    c.moveTo(0, 24, 0, 1e5, false, true);
  }
  if (testTurn) {
    // turn
    controller.clear();
    controller.print(0, 0, "180, 2x 90 degree turns");
    pros::delay(0);
    controller.rumble(".");
    c.setPose(0, 0, 0);

    // 90 clockwise
    c.turnTo(1000, 0, 1e5);
    pros::delay(0);
    // 180
    c.turnTo(-1000, 0, 1e5);
    pros::delay(0);
    // 90
    c.turnTo(0, 1000, 1e5);
  }
  if (testBoomerang) {
    // boomerang
    controller.clear();
    controller.print(0, 0, "forwards 24 inches, 45 turn");
    c.setPose(0, 0, 0);
    c.moveTo(24, 24, 90, 1e5, false, true, 0, 0.1);
  }
  if (testMotionChaining) {
    // move fwd and turn
    c.setPose(0, 0, 0);
    // timeout based motion chaining
    c.moveTo(0, 24, 0, 300);
    c.moveTo(12, 36, 45, 1e5);
  }
}

// skills
void skills() {
  bool shooting = false;
  lemlib::Chassis &c = *skillsChassis;
  pros::Motor_Group &k = *kicker;
  /*
  // init
  c.setPose(52.5, 55, 315);
  // move back to push preloads in
  c.moveTo(66, 28, 0, 2000, false, false, 15, 0.5);
  // move to shoot
  c.moveTo(60, 46, 250, 3000, false, true, 0, 0.2);
  // flip down wings to be in matchload zone
  back_wing_right.set_value(true);
  backRightWingOn = true;
  lemlib::Pose currPose(0, 0, 0);
  if (shooting) {
    const int NUM_BALLS = 44;
    const int NUM_EX = 6;
    k = 127;
    pros::delay(1000);
    currPose = c.getPose();
    // wait while shooting
    // let momentum stop
    pros::delay((NUM_BALLS + NUM_EX) / 2 * 1000 - 1000);
    k = 0;
  } else {
    // 3 second dummy shooting
    pros::delay(2000);
    currPose = c.getPose();
    pros::delay(3e3 - 2000);
  }
  back_wing_right.set_value(false);
  c.setPose(currPose.x, currPose.y, currPose.theta);
  intake = 127;
  // reset to where we started: we haven't moved!
  // collect ball and knock others over
  c.moveTo(20, 20, 252, 2500, false, true, 0, 0.4);
  // move to push over short barrier
  front_wings.set_value(false);
  c.turnTo(20, -40, 800);
  front_wings.set_value(true);
  intake = -127;
  // go quick into
  c.moveTo(20, -40, c.getPose().theta, 1500, false, true, 1e5, 0);
  front_wings.set_value(false);
  // turn to make motion chaining easier
  c.turnTo(40, -40, 600);
  // motion chaining
  c.moveTo(44, -48, 180, 1000, false, true, 10, 0.4);
  c.moveTo(24, -60, 270, 600);
  c.moveTo(-36, -60, 270, 800);
  // score
  c.moveTo(-63, -30, 0, 2500, true, true, 0, 0.6);
  intake = 127;
  c.waitUntilDist(10);
  intake = -127;
  c.waitUntilDist(1e5);
  // back up to get another push
  c.moveTo(-52, -56, 160, 2500, false, false);
  intake = 0;
  // push in with back of bot
  c.moveTo(-60, -20, 180, 2400, false, false, 0, 0.7);
  intake = 0;
  */
  // reset on goal
  // c.setPose(-60, -30, c.getPose().theta);
  c.setPose(-60, -36, 180);
  // move away
  c.moveTo(-48, -48, 180, 1500, false, true, 10, 0.3);
  c.turnTo(-72, -72, 700);
  c.moveTo(-24, -24, 180, 1200, false, false, 0, 0.2);
  back_wing_left.set_value(true);
  back_wing_right.set_value(true);
  // push with back of bot into left side of goal
  c.moveTo(-100, -10, 90, 3000, true, false, 20, 0.9);
  c.waitUntilDist(36);
  back_wing_right.set_value(false);
  c.waitUntilDist(1e5);
  back_wing_left.set_value(false);
  c.setPose(-41, -10, 90);
  // back out
  c.moveTo(-12, -10, 90, 800);
  // get set up for next push
  c.turnTo(-12, 36, 600);
  intake = 127;
  c.moveTo(-12, 36, 0, 1000);
  return;
  back_wing_right.set_value(true);
  // push with back of bot into the right side of goal
  c.moveTo(-44, 18, 90, 3200, false, false, 70, 0.7);
  back_wing_right.set_value(false);
  // back out
  c.moveTo(-12, 18, 90, 2000, false, true, 0, 0);
  c.turnTo(-12, 36, 600);
  // push in with back of bot into center of goal
  back_wing_right.set_value(true);
  back_wing_left.set_value(true);
  c.moveTo(-50, 0, 90, 2600, false, false, 50, 0.9);
  // reset on goal
  c.setPose(-42, 0, c.getPose().theta);
  // go to right side of goal
  c.moveTo(-40, 60, 0, 2300, true, true, 0, 0.9);
  c.waitUntilDist(5);
  back_wing_right.set_value(false);
  c.waitUntilDist(1e5);
  back_wing_left.set_value(false);
  c.turnTo(-60, 30, 800);
  intake = -127;
  // push in
  c.moveTo(-60, 20, 180, 2500, false, true, 0, 0.7);
  // go to hang
  hang.set_value(true);
  intake = 0;
  c.moveTo(-12, 60, 270, 1500, false, false, 0, 0.7);
  c.turnTo(0, 60, 600);
  c.moveTo(12, 60, 90, 1200, true);
  c.waitUntilDist(24);
  hang.set_value(false);
  c.waitUntilDist(1e5);
  hang.set_value(false);
  // celly
  intake = -127;
}