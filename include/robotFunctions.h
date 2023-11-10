void odomScreen();
void flywheelScreen();
double filterJoystickInput(int input);

extern double currFlywheelSpeed;
extern double targetFlywheelSpeed;
void initFlywheelTask();

void handleButtons();