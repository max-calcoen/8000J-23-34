// from https://github.com/kunwarsahni01/Vex-Autonomous-Selector
#pragma once

#include <string>

// selector configuration
#define HUE 360
#define DEFAULT 3
#define AUTONS "Left", "Right", "Test"

namespace selector {

extern int auton;
const char *b[] = {AUTONS, ""};
void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

} // namespace selector
