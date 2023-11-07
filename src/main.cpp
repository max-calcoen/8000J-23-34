#include "main.h"
#include "autoSelect/selection.h"
#include "autons.h"

lemlib::Chassis *chassis = nullptr; // Initialize to nullptr
pros::Controller controller = pros::E_CONTROLLER_MASTER;

void screen() {
  // loop forever
  while (true) {
    lemlib::Pose pose =
        chassis->getPose(); // get the current position of the robot
    pros::lcd::print(0, "x: %f", pose.x);           // print the x position
    pros::lcd::print(1, "y: %f", pose.y);           // print the y position
    pros::lcd::print(2, "heading: %f", pose.theta); // print the heading
    pros::delay(10);
  }
}
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
  // TODO: complete
  pros::Motor left_front(1, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor left_middle(2, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor left_back(3, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor right_front(4, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor right_middle(5, pros::E_MOTOR_GEARSET_06, false);
  pros::Motor right_back(6, pros::E_MOTOR_GEARSET_06, false);

  pros::MotorGroup left({left_front, left_middle, left_back});
  pros::MotorGroup right({right_front, right_middle, right_back});

  lemlib::Drivetrain_t drivetrain{
      &left,  // left drivetrain motors
      &right, // right drivetrain motors
      10,     // track width TODO: calculate
      4.0,    // wheel diameter
      300     // wheel rpm
  };

  // TODO: complete
  // inertial sensor
  pros::Imu inertialSensor(2); // port 2

  // odometry struct
  lemlib::OdomSensors_t odomSensors{
      nullptr, nullptr, nullptr, nullptr,
      &inertialSensor // inertial sensor
  };
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
  chassis->calibrate();
  // create a task to print the position to the screen
  pros::Task screenTask(screen);
  // TODO: tune, also make different autons make starting pose different
  chassis->setPose(0, 0, 0); // X: 0, Y: 0, Heading: 0
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
    pros::delay(20);
  }
}
