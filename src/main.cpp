#include "main.h"
#include "autoSelect/selection.h"
#include "autons.h"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "robotFunctions.h"

pros::Controller controller = pros::E_CONTROLLER_MASTER;

lemlib::Chassis *compChassis = nullptr;   // initialize to nullptr
lemlib::Chassis *skillsChassis = nullptr; // initialize to nullptr

pros::Motor_Group *left_drivetrain = nullptr;  // initialize to nullptr
pros::Motor_Group *right_drivetrain = nullptr; // initialize to nullptr

pros::Motor intake(14, pros::E_MOTOR_GEARSET_06, true);
pros::Motor kicker1(18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor kicker2(13, pros::E_MOTOR_GEARSET_18, true);
pros::Motor_Group *kicker = new pros::Motor_Group({kicker1, kicker2});

pros::Imu inertialSensor(11);
pros::Rotation *odomVertRotation = new pros::Rotation(7);
pros::Rotation *odomHoriRotation = new pros::Rotation(19);
// TODO: measure distance
lemlib::TrackingWheel odomVert(odomVertRotation, 2.75, -0.25, 1);
lemlib::TrackingWheel odomHori(odomHoriRotation, 2.75, -1.5, 1);

pros::Task screen([] {
  while (true) {
    odomScreen();
    pros::delay(10);
  }
});

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  screen.suspend();
  selector::init();
  // temp?
  screen.resume();

  pros::Motor lf(1, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor lm(2, pros::E_MOTOR_GEARSET_06, true);
  pros::Motor lb(3, pros::E_MOTOR_GEARSET_06, true);

  pros::Motor rf(10, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor rm(9, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor rb(8, pros::E_MOTOR_GEARSET_06, false);

  left_drivetrain = new pros::Motor_Group({lf, lm, lb});
  right_drivetrain = new pros::Motor_Group({rf, rm, rb});

  lemlib::Drivetrain_t drivetrain{
      left_drivetrain,  // left drivetrain motors
      right_drivetrain, // right drivetrain motors
      10.7,             // track width (in) TODO: calculate
      2.75,             // wheel diameter
      450,              // wheel rpm
      30,               // chase power
  };

  // no tracking wheels
  lemlib::OdomSensors_t odomSensors{nullptr, nullptr, nullptr, nullptr,
                                    &inertialSensor};
  lemlib::OdomSensors_t skillsOdomSensors{&odomVert, nullptr, &odomHori,
                                          nullptr, &inertialSensor};

  // https://lemlib.github.io/LemLib/md_docs_tutorials_3_tuning_and_moving.html
  lemlib::ChassisController_t lateralController{
      25,   // kP
      15,   // kD
      0.25, // smallErrorRange
      100,  // smallErrorTimeout
      0.75, // largeErrorRange
      200,  // largeErrorTimeout
      20    // slew rate
  };
  lemlib::ChassisController_t angularController{
      2.5, // kP
      4,   // kD
      1.5, // smallErrorRange
      150, // smallErrorTimeout
      4.0, // largeErrorRange
      250, // largeErrorTimeout
      30   // slew rate
  };

  // create the chassis and calibrate
  compChassis = new lemlib::Chassis(drivetrain, lateralController,
                                    angularController, odomSensors);

  skillsChassis = new lemlib::Chassis(drivetrain, lateralController,
                                      angularController, skillsOdomSensors);
  compChassis->calibrate();
  // skillsChassis->calibrate();
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
    redDefensive();
    break;
  case 2:
    redOffensive();
    break;
  case 3:
    redTest();
    break;
  case -1:
    blueDefensive();
    break;
  case -2:
    blueOffensive();
    break;
  case -3:
    blueTest();
    break;
  }
}

/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is
 * enabled via the Field Management System or the VEX Competition Switch in
 * the operator control mode.
 *
 * If no competition control is connected, this function will run
 * immediately following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */
void opcontrol() {
  int prevLeftSpeed = 0;
  right_drivetrain->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);
  left_drivetrain->set_brake_modes(pros::E_MOTOR_BRAKE_COAST);

  while (true) {
    handleButtons();

    const double DRIVE_SENS = 1.0;
    const double TURN_SENS = 1.0;

    int left = controller.get_analog(ANALOG_LEFT_Y);
    int right = controller.get_analog(ANALOG_RIGHT_X);

    int targetLeftSpeed = left * DRIVE_SENS;
    int targetTurnSpeed = right * TURN_SENS;

    // TODO: tune
    int forward = filterJoystickInput(targetLeftSpeed, 1);
    int turn = filterJoystickInput(targetTurnSpeed, 1.6);

    // arcade drive calculations
    int leftspeed = forward + turn;
    int rightspeed = forward - turn;

    // limit speeds to valid range
    leftspeed = std::max(-127, std::min(127, leftspeed));
    rightspeed = std::max(-127, std::min(127, rightspeed));

    left_drivetrain->move(leftspeed);
    right_drivetrain->move(rightspeed);

    pros::delay(100);
    prevLeftSpeed = forward;
  }
}