#include "api.h"

extern pros::ADIDigitalOut front_wings;
extern pros::ADIDigitalOut back_wing_left;
extern pros::ADIDigitalOut back_wing_right;
extern pros::ADIDigitalOut hang;

extern bool frontWingsOn;
extern bool backLeftWingOn;
extern bool backRightWingOn;
extern bool hangUp;

double filterJoystickInput(int input, double scale);
void handleButtons();

void odomScreen();