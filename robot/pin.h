// Some helper functions to access the jetson GPIO pins directly.
// I used this originally to control stepper motor controllers, but since I replaced those
// with BLDC motors this code is unused.
// I've left this code in just in case someone needs it in the future.

#pragma once

bool pin_init();
void pin_close();

enum PinMode
{
	pin_mode_out,
	pin_mode_in,
};
bool pin_set_mode(unsigned int gpio, PinMode mode);
bool pin_set(unsigned int gpio, unsigned int value);
//bool pin_export(unsigned int gpio);
