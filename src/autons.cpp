#include "lemlib/chassis/chassis.hpp"
#include "main.h"
#include "pros/misc.h"
#include "robotFunctions.h"

ASSET(red_offensive_1_txt)
ASSET(skills1_txt)

/*
 * helpful resources
 * path planner: https://path.jerryio.com/
 */

void redOffensive() {
  // lemlib::Chassis &c = *compChassis;
  // COMP CHASSIS
  lemlib::Chassis &c = *compChassis;
  // init
  c.setPose(-12, 60, 90);
  intake = 127;
  // move to pick up ball under center bar
  c.moveTo(-10, 60, 90, 300);
  // wait for ball to intake
  pros::delay(100);
  // back up and get ball out of corner
  c.follow(red_offensive_1_txt, 1650, 8, true, false);
  c.waitUntilDist(34);
  // put down back wing to get ball out of corner
  back_wing_left.set_value(true);
  backLeftWingOn = true;
  // pull wing up so bot doesn't get stuck on wall
  c.waitUntilDist(45.5);
  back_wing_left.set_value(false);
  backLeftWingOn = false;
  c.waitUntilDist(1e5);
  // back up from goal (move forwards)
  c.moveTo(-48, 48, 45, 1e5);
  c.turnTo(-60, 30, 1e5);
  // push ball in intake into the goal
  c.moveTo(-56, 30, 180, 1000, true);
  c.waitUntilDist(3);
  intake = -127;
  c.waitUntilDist(1e5);
  // back up
  c.moveTo(-50, 50, 230, 1e5, false, false);
  // turn to next ball
  c.turnTo(-2, c.getPose().y, 1e5);
  intake = 127;
  // collect
  c.moveTo(-2, 40, 120, 1e5);
  // reset
  c.setPose(-12, 26, c.getPose().theta);
  // roll ball to goal
  c.turnTo(-25, 19, 1500);
  c.moveTo(-25, 19, 250, 2000, true);
  c.waitUntilDist(10);
  intake = -127;
  c.waitUntilDist(1e5);
  // turn to next ball and collect
  c.turnTo(3, 0, 1e5);
  intake = 127;
  c.moveTo(3, 0, 135, 1e5);
  // push rest into goal
  c.turnTo(-36, c.getPose().y, 2200);
  front_wings.set_value(true);
  frontWingsOn = true;
  intake = -127;
  c.moveTo(-36, c.getPose().y, 270, 1500, false);
  // back up
  c.moveTo(-35, c.getPose().y, 270, 1e5, false, false);
  // go touch bar
  c.moveTo(-3, 40, 45, 1e5, false, true, 0, 0.1);
}
void redDefensive() {
  lemlib::Chassis &c = *compChassis;
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
}

void blueOffensive() {}
void blueDefensive() {}

// ODOM-BASED PID MOVEMENT + BOOMERANG TEST
void redTest() {
  // test bools
  bool testFwd = false;
  bool testBwd = false;
  bool testTurn = false;
  bool testBoomerang = true;
  bool testMotionChaining = false;
  // COMP CHASSIS
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

void blueTest() {}

// skills
void skills() {
  controller.rumble("--");
  bool aggressive = true;
  bool shooting = false;
  lemlib::Chassis &c = *skillsChassis;
  pros::Motor_Group &k = *kicker;
  if (aggressive) {
    // init
    c.setPose(52.5, 53, 315);
    // move back to push preloads in
    c.moveTo(63, 30, 0, 2300, false, false, 5, 0.2);
    // move to shoot
    c.moveTo(56, 44, 250, 1e5);
    // flip down wings to be in matchload zone
    back_wing_right.set_value(true);
    backRightWingOn = true;
    if (shooting) {
      k = 127;
      const int NUM_BALLS = 44;
      const int NUM_EX = 4;
      // wait while shooting
      pros::delay((NUM_BALLS + NUM_EX) / 2 * 1000);
      k = 0;
    } else {
      // 3 second dummy shooting
      pros::delay(3e3);
    }
    back_wing_right.set_value(false);
    backRightWingOn = false;
    intake = 127;
    // collect ball and knock others over
    c.moveTo(12, 24, 270, 1e5);
    // move to push over short barrier
    front_wings.set_value(false);
    c.turnTo(14, -40, 1e5);
    front_wings.set_value(true);
    intake = -127;
    c.moveTo(14, -40, 180, 1e5);
    front_wings.set_value(false);
    // turn to make motion chaining easier
    c.turnTo(-40, -36, 1e5);
    // motion chaining
    c.moveTo(40, -48, 180, 2500, false, true, 10, 0.4);
    c.moveTo(24, -60, 270, 2500);
    c.moveTo(-36, -60, 270, 1e5);
    // score
    c.moveTo(-60, -32, 0, 2500, false, true, 0, 0.5);
    front_wings.set_value(false);
    // back out
    c.moveTo(-36, -33, 270, 2500, false, false, 0, 0.1);
    // push in pt1
    c.moveTo(-12, -24, 180, 2500, false, false);
    back_wing_left.set_value(true);
    back_wing_right.set_value(true);
    // push in pt2
    c.moveTo(-36, -12, 90, 2500, false, false);
    // move back to get another angle
    c.moveTo(-12, -12, 270, 1e5);
    c.turnTo(-12, 24, 1e5);
    c.moveTo(-12, 24, 0, 1e5);
    // back up to push in
    c.moveTo(-36, 12, 90, 3000, false, false);
    c.waitUntilDist(10);
    back_wing_left.set_value(true);
    back_wing_right.set_value(true);
    c.waitUntilDist(1e5);
    c.moveTo(-12, 36, 0, 1e5);
  }
}