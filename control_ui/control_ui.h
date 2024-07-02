
#pragma once
#include "../common/common.h"
#include <vector>
struct Plot;

struct MonitorDataEx : MonitorData
{
	// Some additional variables for simulation and visualization stuff.
	// This is not used on the robot, only in control ui.
	bool balance_control_is_quiet = false;
	
	float coi_world_pos_sim = 0;
	float coi_world_vel_sim = 0;
	float coi_world_acc_sim = 0;
	float coi_theta_acc_sim = 0;

	float side_angle_motor_calc = 0;
	float side_angle_motor_d_calc = 0;
	float average_leg_pos_calc = 0;
	float average_leg_pos_d_calc = 0;
	float side_angle_motor_force_calc = 0;
	float average_leg_pos_force_calc = 0;
	float force_l_calc = 0;
	float force_r_calc = 0;
	float acc_l_calc = 0;
	float acc_r_calc = 0;

	float theta_train = 0;
	s64 display_time = 0;
	float imu_gravity_sim_x = 0;
	float imu_gravity_sim_y = 0;
	float imu_gravity_sim_z = 0;
	float imu_x_angle_sim = 0;
	float target_angle_l = 0;
	float target_angle_r = 0;
	float squat_length_sim = 0;
	float balance_control_action_calc = 0;
	float theta_bias_vel_user = 0;
	float leg_current_landing_sim[2] = {0};
	float leg_vel_target_odrv_sim[2] = {0};
	float leg_rel_angle_target_odrv_sim[2] = {0};
	/*float x_angle_from_hip_l = 0;
	float x_angle_from_hip_r = 0;
	float x_angle_from_hip = 0;*/
	//float foot_vel_coarse_sim[2] = {0};
};


extern float dpi_scaling;
extern int window_size_x, window_size_y;

extern Plot* main_plot;

void draw_ui_main_2();
std::vector<MonitorDataEx>& get_controller_history();
