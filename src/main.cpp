// TODO: test
// flywheel with grapher
// odom with lemlib
// drive

// CHECKLIST:
// name            coded       tested
// drive           yes         no
// flywheel pid    yes         no
// pid             yes         no
// auton selector  yes         no
// path following  no          no
// intake          yes         no
// autons          no          no

#include "main.h"
#include "autoSelect/selection.h"
#include "autons.h"
#include "pros/adi.h"
#include "robotFunctions.h"

pros::Controller controller = pros::E_CONTROLLER_MASTER;

lemlib::Chassis *chassis = nullptr; // initialize to nullptr

pros::Motor_Group *left_drivetrain = nullptr;  // initialize to nullptr
pros::Motor_Group *right_drivetrain = nullptr; // initialize to nullptr

pros::Motor *intake = new pros::Motor(1, pros::E_MOTOR_GEARSET_06, true);
pros::Motor *flywheel = new pros::Motor(2, pros::E_MOTOR_GEARSET_06, true);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // selector::init();
  // // TODO: figure out how to switch from selector in pre-comp to lcd in comp
  // TODO: branch selector and make better auton selector
  pros::lcd::initialize();

  pros::Motor lf(13, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor lm(12, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor lb(11, pros::E_MOTOR_GEARSET_06, true);

  pros::Motor rf(16, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor rm(15, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor rb(14, pros::E_MOTOR_GEARSET_06, true);

  left_drivetrain = new pros::Motor_Group({lf, lm, lb});
  right_drivetrain = new pros::Motor_Group({rf, rm, rb});

  lemlib::Drivetrain_t drivetrain{
      left_drivetrain,  // left drivetrain motors
      right_drivetrain, // right drivetrain motors
      10,               // track width (in) TODO: calculate
      4.0,              // wheel diameter
      300               // wheel rpm
  };

  // inertial sensor
  pros::Imu inertialSensor(18); // port 2

  // odometry struct
  lemlib::OdomSensors_t odomSensors{
      nullptr, nullptr, nullptr, nullptr,
      &inertialSensor // inertial sensor
  };
  // https://lemlib.github.io/LemLib/md_docs_tutorials_3_tuning_and_moving.html
  // TODO: tune
  // forward/backward PID
  lemlib::ChassisController_t lateralController{
      8,   // kP
      30,  // kD
      1,   // smallErrorRange
      100, // smallErrorTimeout
      3,   // largeErrorRange
      500, // largeErrorTimeout
      5    // slew rate
  };
  // TODO: tune
  // turning PID
  lemlib::ChassisController_t angularController{
      4,   // kP
      40,  // kD
      1,   // smallErrorRange
      100, // smallErrorTimeout
      3,   // largeErrorRange
      500, // largeErrorTimeout
      0    // slew rate
  };

  // create the chassis
  chassis = new lemlib::Chassis(drivetrain, lateralController,
                                angularController, odomSensors);
  // callibrate chassis
  // chassis->calibrate();
  // create screen task
  // pros::Task screenTask(flywheelScreen);
  // TODO: different autons have different starting poses
  chassis->setPose(0, 0, 0);
}

/**
 * Runs while the robot is in the disabled state of Field Management System
 * or the VEX Competition Switch, following either autonomous or opcontrol.
 * When the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  switch (selector::auton) {
  case 0:
    skills();
    break;
  case 1:
    redLeft();
    break;
  case 2:
    redRight();
    break;
  case 3:
    redTest();
    break;
  case -1:
    blueLeft();
    break;
  case -2:
    blueRight();
    break;
  case -3:
    blueTest();
    break;
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  while (true) {
    handleButtons();

    const double DRIVE_SENS = 1.0;
    const double TURN_SENS = 0.8;

    pros::lcd::set_text(1,
                        std::to_string(controller.get_analog(ANALOG_LEFT_Y)));
    pros::lcd::set_text(2,
                        std::to_string(controller.get_analog(ANALOG_RIGHT_Y)));

    int forward = filterJoystickInput(controller.get_analog(ANALOG_LEFT_Y) *
                                      DRIVE_SENS); // forward/backward
    int turn = filterJoystickInput(controller.get_analog(ANALOG_RIGHT_X) *
                                   TURN_SENS); // turning

    int leftspeed = forward + turn;
    int rightspeed = forward - turn;

    leftspeed = std::max(-127, std::min(127, leftspeed));
    rightspeed = std::max(-127, std::min(127, rightspeed));

    left_drivetrain->move(leftspeed);
    right_drivetrain->move(rightspeed);

    pros::delay(20);
  }
}