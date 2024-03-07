#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "main.h"
#include "pros/misc.h"
#include "robotFunctions.h"

#include <iomanip>
#include <sstream> // For std::ostringstream to convert numbers to strings.

ASSET(skills1_txt)

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
  lemlib::Chassis &c = *skillsChassis;

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
  controller.clear();
  bool shooting = true;
  lemlib::Chassis &c = *skillsChassis;
  pros::Motor_Group &k = *kicker;
  // init
  c.setPose(52.5, 55, 315);
  // move back to push preloads in
  c.moveTo(66, 30, 0, 2300, false, false, 15, 0.40);
  // move to shoot
  c.moveTo(60, 46, 255, 1e5, false, true, 0, 0.2);
  // flip down wings to be in matchload zone
  back_wing_right.set_value(true);
  backRightWingOn = true;
  lemlib::Pose currPose(0, 0, 0);
  std::ostringstream beforeShootStream;
  beforeShootStream << "" << std::fixed << std::setprecision(1) << c.getPose().x
                    << "," << c.getPose().y << "," << c.getPose().theta;
  controller.set_text(
      0, 0, beforeShootStream.str().c_str()); // Print pose before shooting
  if (shooting) {
    const int NUM_BALLS = 44;
    const int NUM_EX = 16;
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
  std::ostringstream afterShootStream;
  afterShootStream << "" << std::fixed << std::setprecision(1) << c.getPose().x
                   << "," << c.getPose().y << "," << c.getPose().theta;
  controller.set_text(
      1, 0, afterShootStream.str().c_str()); // print pose after shooting
  intake = 127;
  // reset to where we started: we haven't moved!
  // collect ball and knock others over
  c.moveTo(16, 20, 250, 2500, false, true, 0, 0.4);
  // move to push over short barrier
  front_wings.set_value(false);
  c.turnTo(16, -40, 1000);
  front_wings.set_value(true);
  intake = -127;
  // go quick into
  c.moveTo(16, -40, c.getPose().theta, 1500, false, true, 1000, 0);
  front_wings.set_value(false);
  // turn to make motion chaining easier
  c.turnTo(40, -40, 600);
  // motion chaining
  c.moveTo(40, -48, 180, 1000, false, true, 10, 0.4);
  c.moveTo(24, -61, 270, 800);
  c.moveTo(-36, -61, 270, 1000);
  front_wings.set_value(true);
  // score
  c.moveTo(-64, -30, 0, 2000, false, true, 0, 0.5);
  front_wings.set_value(false);
  // back up to get another push
  c.moveTo(-48, -48, 270, 1000, false, false);
  c.turnTo(-48, -56, 1000);
  // push in with back of bot
  c.moveTo(-56, -20, 180, 2400, false, false);
  c.moveTo(-48, -56, 160, 1500);
  c.moveTo(-56, -20, 180, 2400, false, false);
  intake = 0;
  // reset on goal
  c.setPose(-56, -30, c.getPose().theta);
  c.moveTo(-56, -31, 275, 1000, false, false);
  // swing motion chaining: push in from left
  c.moveTo(-24, -36, 270, 1400, false, false, 3, 0.6, 90);
  back_wing_left.set_value(true);
  c.moveTo(-12, -28, 180, 1200, false, false, 3, 0.6, 90);
  c.moveTo(-42, -12, 90, 3000, false, false, 3, 0.6, 90);
  // push in from center
  // move out
  c.moveTo(-12, -6, 90, 2000, true);
  c.waitUntilDist(10);
  back_wing_left.set_value(false);
  c.waitUntilDist(1e5);
  // slow no motion chaining
  // push in
  back_wing_left.set_value(true);
  back_wing_right.set_value(true);
  c.moveTo(-44, 0, 90, 4000, false, false, 0, 0.1, 80);
  // move away
  c.moveTo(-12, -12, 90, 1e5, true);
  c.waitUntilDist(10);
  back_wing_left.set_value(false);
  back_wing_right.set_value(false);
  c.waitUntilDist(1e5);
  // push in from right
  c.turnTo(-12, 36, 1000);
  c.moveTo(-12, 36, 0, 1800);
  back_wing_right.set_value(true);
  c.moveTo(-42, 18, 90, 4000, false, false, 0, 0.2, 80);
  back_wing_right.set_value(false);
  // reset on goal
  c.setPose(-42, 14, c.getPose().theta);
  // right side
  // move out motion chaining
  c.moveTo(-12, 24, 0, 1000);
  c.moveTo(-36, 36, 270, 1000);
  front_wings.set_value(true);
  c.moveTo(-48, 48, 315, 1000, false, true, 0, 0.1);
  c.moveTo(-60, 30, 180, 3000);
  c.moveTo(-48, 48, 225, 1000, false, false);
  front_wings.set_value(false);
  c.turnTo(24, 72, 1300);
  c.moveTo(-60, 30, 0, 3000, false, false, 10000);
  c.moveTo(-48, 48, 30, 1000, false, true);
  c.moveTo(-60, 30, 0, 3000, false, false, 10000);
}