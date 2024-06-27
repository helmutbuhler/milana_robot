#pragma once


bool leaning_init();
bool leaning_update();
void leaning_close();

void calculate_rotation_vel_range(float h, float v, float imu_side_angle, float& r_min, float& r_max);
