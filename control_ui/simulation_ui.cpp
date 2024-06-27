// With this you can simulate the robot and compare it to recorded data.
// This code is a bit messy. Large parts of the robot main loop and the ODrive Control Loop are copied here.
// When I started implementing this, it seemed infeasable to separate the sensor retrieval stuff from
// the logic stuff in the robot loop, but maybe that would have been a better choice to prevent this
// code duplication.
// Anyway, the code duplication is limited to this file, and the simulation stuff is not essential to the
// project.
// For more details on the actual simulation, see pendulum_simulation.h
// To switch between the regular view and the simulation view, press S.

#define _CRT_SECURE_NO_WARNINGS
#include "simulation_ui.h"
#include "control_ui.h"
#include "../common/helper.h"
#include "../common/geometry.h"
#include "pendulum_simulation.h"

#include <algorithm>

#include "GLFW/glfw3.h"

#include "imgui.h"
#include "plot.h"

#define NEW_LANDING

using namespace std;

struct SimMonitorData
{
	s64 display_time = 0;
	float balance_control_vel_output = 0;
	float balance_control_vel_user = 0;
	float fake_x_angle_delta_vel_user = 0;

	float imu_x_angle = 0, theta_dot = 0, theta_acc = 0;
	float imu_x_angle_interframe = 0;
	float sim_target_x_angle_delta = 0; // This angle is added in the pendulum simulation to fake a deviation in the com calculation
	float balance_control_target_angle = 0;
	float balance_control_target_angle_vel = 0;

	// leg control
	float common_leg_pos_target = 0;      // input_pos    on ODrive
	float common_leg_pos_target_odrv = 0; // pos_setpoint on ODrive
	float common_leg_pos = 0;
	float common_leg_pos_dot = 0;
	float common_leg_pos_dot_target_odrv = 0;
	float leg_integrator = 0;
	float common_leg_vel_target = 0;
	float leg_current = 0;
	JumpPhase jump_phase = JP_None;
	float post_landing_timer = 1;
	float l_max_vel = 0;
	float l_vel_target_delta = 0;
	float l_vel_target = 0;
	float l_leg_height_pos = 0;
	float l_leg_height_vel = 0;
	float l_hip_angle = 0;
	float low_vel_timer = 0;
	float low_vel2_timer = 0;

	// hip control
	float common_hip_pos_target = 0;
	float common_hip_pos_sensor = 0;
	float common_hip_pos_sensor_dot = 0;
	float common_servo_vel = 0;

	float squat_length = 0;

	float error_p = 0; // pid error
	float error_d = 0;
	float vel_p = 0; // pid contributions to balance_control_vel_output
	float vel_d = 0;
	float vel_pos = 0, balance_control_integrator = 0;
	float vel_backcalculation = 0;
	float balance_control_standing_action_tolerance_timer = 0;
	float t_correction_angle = 0;
	float t_correction_angle_raw = 0;
	//float balance_control_body_action = 0;
	//float balance_control_body_action_delta = 0;
	bool is_standing = false;
	float balance_control_pos_user = 0;
	/*float balance_control_pos_output = 0;
	float balance_control_vel_output = 0; // target velocity
	float balance_control_vel_user = 0;
	float balance_control_vel_quasi = 0;*/
	float balance_control_vel_quasi_smooth = 0;
	/*float balance_control_rotation_error_bias = 0;
	float balance_control_rotation_error = 0;
	float balance_control_rotation_vel = 0;
	float balance_control_vel_extra_factor = 1.0f;*/
	float balance_control_delta_pos = 0;
	float balance_control_delta_vel = 0;
	bool balance_control_force_still = false;
	float balance_control_gain_p = 0;
	float balance_control_gain_i = 0;
	float balance_control_gain_d = 0;
	float balance_control_gain_vel_high = 0;
	float balance_control_gain_vel_low = 0;
	float balance_control_gain_backcalculation = 0;
	float balance_control_position_control_error = 0;
	bool balance_control_enable_position_control = false;

	// foot control:
	float foot_pos = 0;
	float foot_vel_target = 0;
	float foot_vel_coarse = 0;
	float foot_vel_coarse_interframe = 0;
	float foot_current = 0;
	float foot_integrator = 0;

	float rubber_pos = 0;
	float rubber_vel_interframe = 0;

	float imu_sensor_to_floor_pos_x = 0;
	float imu_sensor_to_floor_pos_y = 0;
	float imu_sensor_to_floor_vel_x = 0;
	float imu_sensor_to_floor_vel_y = 0;
	float imu_sensor_to_floor_acc_x = 0; // in g-force
	float imu_sensor_to_floor_acc_y = 0; // in g-force

	float coi_world_pos = 0; // center of inertia in forward direction in world
	float coi_world_vel = 0;
	float coi_world_acc = 0; // in g-force
	float coi_theta_acc = 0;
};



struct SimControlData
{
	int target_delta_time_ms = 18;

	// foot control:
	float foot_control_vel_limit = 15;
	float foot_control_accel_limit = 50;
	float foot_control_current_lim = 30;

	// hip
	float hip_sensor_angle_range = 1.847749f; // todo: rename
	
	float balance_control_target_angle_vel_gain = 0.02f;
	float balance_control_target_angle_vel_smooth = 0.97f;
#if 1
	float balance_control_gain_p = 10;
	float balance_control_gain_i = 57;
	float balance_control_gain_d = 0.2f;
	float balance_control_gain_vel_high = 2.5f;
	float balance_control_gain_vel_low = 3.5f;
	float balance_control_gain_backcalculation = 40;
	float max_balance_control_integrator = 100;
	float balance_control_max_theta_integrator = 0.5f;
	float stepper_avg_smooth = 0.01f;
	float balance_control_vel_user_acceleration = 20;
	bool balance_control_use_coi_vel = true;
	float balance_control_vel_acceleration_limit = 0.25f;
#else
	float balance_control_gain_p = 14;
	float balance_control_gain_i = 60;
	float balance_control_gain_d = 0.2f;
	float balance_control_gain_vel_high = 3;
	float balance_control_gain_vel_low = 5;
	float balance_control_gain_backcalculation = 40;
	float max_balance_control_integrator = 100;
	float balance_control_max_theta_integrator = 0.5f;
	float stepper_avg_smooth = 0.01f;
	float balance_control_vel_user_acceleration = 20;
	bool balance_control_use_coi_vel = false;
	float balance_control_vel_acceleration_limit = 0.3f;
#endif
	float balance_control_vel_target = 0;
	//bool  balance_control_force_still = false;
	//float balance_control_quasi_factor = 1.5f;
	float balance_control_quasi_factor = 0;
	float balance_control_vel_user_max = 1.5f;
	float balance_control_standing_theta_tolerance = 0.8f;
	float balance_control_standing_action_tolerance = 5;
	float balance_control_standing_action_tolerance_time = 1.5f;
	float balance_control_gain_vel_landing_warmup_time = 0;
	
	float balance_control_rotation_vel_user_max = 1;
	float balance_control_rotation_vel_user_target = 0;
	float balance_control_rotation_vel_acc = 2;
	float balance_control_rotation_error_factor = 0.5f;

	//float balance_control_body_action_delta_smooth = 0.98f;
	float balance_control_error_d_smooth = 0.9f;
	float theta_dot_smooth = 0.4f;
	float theta_acc_smooth = 0;
	float balance_control_vel_quasi_smooth_factor = 0.999f;
	float balance_control_vel_quasi_smooth_max_acc = 10;
	float body_action_factor_0 = 20;
	float body_action_factor_1 = 0;
	float body_action_delta_factor = 0.047f;

	int balance_control_position_control_after_frame = 0;
	float balance_control_position_control_target = 0;
	float balance_control_position_control_max_vel = 1.5f;
	float balance_control_position_control_gain_p_high = 1.5f;
	float balance_control_position_control_gain_d_high = 0.3f;
	float balance_control_position_control_gain_p_low = 2.0f;
	float balance_control_position_control_gain_d_low = 0.5f;
	
	
	float control_vel_factor = 1;
	float fake_x_angle_delta_vel = 0.03f;
	float sim_target_x_angle_delta = 0;
	
	float common_leg_vel_user_max = 1.3f;
	float stand_control_min_leg_angle = -0.65f, stand_control_max_leg_angle = 0.97f;
	float vel_gain = 13;
	float pos_gain = 10;
	float vel_integrator_gain = 30;
	float vel_limit = 800;
	float input_filter_bandwidth = 20;
	float current_lim = 20;

	// landing
#ifdef NEW_LANDING
	float l_pos_zero_delta = 100;
	float l_pos_zero_max = 0.97f;
	float l_vel_gain = 50.0f;
	float l_pos_gain = 0;
	float l_base_angle_delta = -0.4f;
	float l_max_time = 0.5f;
#else
	float l_pos_zero_delta = 0.02f;
	float l_pos_zero_max = 0.82f;
	float l_vel_gain = 1.0f;
	float l_pos_gain = 30;
#endif
	float l_vel_integrator_gain = 0;
	float l_vel_limit = 300;
	float pos_err_sync_gain = 100;
#ifdef NEW_LANDING
	float landing_fixed_foot_vel_time = 0.1f;
	float landing_fixed_foot_vel = 0.2f;
#else
	float landing_fixed_foot_vel_time = 0.15f;
	float landing_fixed_foot_vel = 1.5f;
#endif
	float f_current_lim = 30; // current limit for falling and landing phase

	float leg_control_motor_kv = 100;

	// hip
	float common_servo_vel_user_max = 1;
	
	// foot
	float foot_control_vel_gain = 8;
	float foot_control_vel_integrator_gain = 300;
	float foot_control_motor_kv = 280*0.5f; // not sure why this is half

	int num_sim_iterations = 50;
	float vel_sensor_delay = 0.60f;
	float imu_sensor_delay = 0.35f;
	float hip_motor_p_gain = 80;
	float hip_motor_d_gain = 10;
	float leg_motor_p_gain = -40 * (tau / leg_gear_reduction);
	float leg_motor_d_gain = -2 * (tau / leg_gear_reduction);

	float imu_sensor_to_floor_vel_smooth_factor = 0.94f*0;
	float imu_sensor_to_floor_acc_smooth_factor = 0.94f;

	float sim_squat_body_cog_x = -2.908f;
	float sim_squat_body_cog_y = -87.078f;
	float sim_squat_body_mass = 1.467f;
	float sim_squat_body_inertia = 0.027f;

	float sim_squat_upper_leg_cog_y = 60.419f;
	float sim_squat_upper_leg_mass = 1.6f;
	float sim_squat_upper_leg_inertia = 0.027f;

	float sim_squat_lower_leg_cog_y = 66.193f;
	float sim_squat_lower_leg_mass = 1.06f;
	float sim_squat_lower_leg_inertia = 0.027f;

	float sim_squat_wheel_mass = 0;
	float sim_squat_wheel_inertia = 0.001f;

	float sim_squat_rubber_inertia = 0.001f;
	float sim_squat_rubber_factor = 5.5f;
	float sim_squat_body_inertia_factor = 1;
	float sim_squat_body_mass_factor = 1;

};

struct SimRealData
{
	SimMonitorData smd;
	SimMonitorData rmd;
};

SimControlData scd;
extern ControlData cd;
static std::vector<SimRealData> history;
bool use_control_gains = false;

SimMonitorData convert_history_item(const MonitorDataEx& md, const MonitorData* md_last)
{
	if (md.monitor_data_version < 9)
	{
		// This didn't exist until this version
		scd.balance_control_target_angle_vel_gain = 0;
	}
	SimMonitorData smd;
	smd.balance_control_vel_output = md.balance_control_vel_output;
	smd.balance_control_vel_user = md.balance_control_vel_user*md.balance_control_vel_extra_factor;
	smd.balance_control_vel_quasi_smooth = md.balance_control_vel_quasi_smooth;
	//smd.balance_control_pos_user = md.balance_control_pos_user;
	smd.t_correction_angle = md.t_correction_angle;
	smd.error_p = md.error_p;
	smd.error_d = md.error_d;
	smd.vel_p = md.vel_p;
	if (md.monitor_data_version >= 4)
		smd.vel_d = md.vel_d;
	smd.vel_pos = md.vel_pos;
	smd.balance_control_integrator = md.balance_control_integrator;
	smd.vel_backcalculation = md.vel_backcalculation;
	smd.imu_x_angle            = md.imu_x_angle;
	smd.imu_x_angle_interframe = md.imu_x_angle;
	
	smd.common_hip_pos_target = md.common_hip_pos_target;
	//smd.common_hip_pos_target = md.common_servo_sensor;
	smd.common_hip_pos_sensor = md.common_hip_pos_sensor;
	smd.common_leg_pos_target = md.common_leg_pos_target;
	smd.common_leg_pos_target_odrv = md.common_leg_pos_target;
	//smd.common_leg_pos_target = md.common_leg_pos;
	smd.common_leg_pos = md.common_leg_pos_sensor;
	if (md.monitor_data_version >= 9)
	{
		smd.balance_control_target_angle = md.balance_control_target_angle;
		smd.balance_control_target_angle_vel = md.balance_control_target_angle_vel;
		smd.squat_length = md.balance_control_squat_length;
	}
	else
		smd.balance_control_target_angle = calculate_squat_angle(md.common_hip_pos_sensor, md.common_leg_pos_sensor, &smd.squat_length);
	//if (md_last)
	//	smd.common_leg_vel_target = (md.common_leg_pos_target - md_last->stand_control_angle) / md.delta_time;
	smd.common_leg_vel_target = md.common_leg_vel_target;

	smd.common_servo_vel = md.common_servo_vel;

	smd.foot_pos = (md.foot_pos[0] - md.foot_pos[1]) * 0.5f;
	smd.foot_vel_target = (md.foot_vel_target[0] - md.foot_vel_target[1]) * 0.5f;
	smd.foot_vel_coarse = (md.foot_vel_coarse[0] - md.foot_vel_coarse[1]) * 0.5f;
	smd.foot_vel_coarse_interframe = smd.foot_vel_coarse;
	smd.foot_current = (md.foot_current_target[0] - md.foot_current_target[1]) * 0.5f;
	//smd.foot_acc = md.foot_acc;
	
	smd.rubber_pos = smd.foot_pos;
	smd.rubber_vel_interframe = smd.foot_vel_coarse_interframe;
	
	smd.jump_phase = md.jump_phase;
	smd.post_landing_timer = md.post_landing_timer;
	smd.leg_current = (-md.leg_current_target[0] + md.leg_current_target[1]) * 0.5f;
	smd.leg_integrator = (-md.leg_integrator[0] + md.leg_integrator[1]) * 0.5f;

	smd.display_time = md.display_time;
	smd.balance_control_delta_vel = (md.balance_control_delta_vel_l + md.balance_control_delta_vel_r) * 0.5f;
	smd.balance_control_delta_pos = (md.balance_control_delta_pos_l + md.balance_control_delta_pos_r) * 0.5f;
	smd.is_standing = md.is_standing;
	if (md.monitor_data_version >= 1)
	{
		smd.balance_control_force_still = md.balance_control_force_still;
		smd.balance_control_gain_p = md.balance_control_gain_p;
		smd.balance_control_gain_i = md.balance_control_gain_i;
		if (md.monitor_data_version >= 4)
			smd.balance_control_gain_d = md.balance_control_gain_d;
		smd.balance_control_gain_vel_high = md.balance_control_gain_vel_high;
		if (md.monitor_data_version >= 5)
			smd.balance_control_gain_vel_low = md.balance_control_gain_vel_low;
		smd.balance_control_gain_backcalculation = md.balance_control_gain_backcalculation;
	}
	else
	{
		smd.balance_control_gain_p = 14;
		smd.balance_control_gain_i = 60;
		smd.balance_control_gain_d = 0;
		smd.balance_control_gain_vel_high = 5;
		smd.balance_control_gain_backcalculation = 40;
	}

	smd.imu_sensor_to_floor_pos_x = md.imu_sensor_to_floor_pos_x;
	smd.imu_sensor_to_floor_pos_y = md.imu_sensor_to_floor_pos_y;
	smd.imu_sensor_to_floor_vel_x = md.imu_sensor_to_floor_vel_x;
	smd.imu_sensor_to_floor_vel_y = md.imu_sensor_to_floor_vel_y;
	smd.imu_sensor_to_floor_acc_x = md.imu_sensor_to_floor_acc_x;
	smd.imu_sensor_to_floor_acc_y = md.imu_sensor_to_floor_acc_y;
	
	if (md.monitor_data_version < 28)
	{
		float th_pos = md.error_p;// Using the value from last frame seems to work better
		float leg_pos = smd.common_leg_pos;
		float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
		float hip_angle = hip_angle_0 + scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
		float foot_pos  =  smd.foot_pos;
		float coi_x;
		get_com_world_position(md.imu_x_angle, th_pos,
				hip_angle, leg_angle, foot_pos,
				hip_angle, leg_angle, foot_pos,
				0, 0,
				&coi_x, 0,
				&smd.coi_theta_acc);
		
		smd.coi_world_pos = coi_x;
	}
	else
	{
		smd.coi_world_pos = md.coi_world_pos;
		smd.coi_world_vel = md.coi_world_vel;
		smd.coi_world_acc = md.coi_world_acc;
		smd.coi_theta_acc = md.coi_theta_acc;
	}
	return smd;
}

static int display_time_to_history_index(s64 time)
{
	int start = 0;
	int end = (int)history.size()-1;
	while (true)
	{
		if (start == end)
			return start;
		int middle = (start+end)/2;
		if (history[middle].smd.display_time < time)
			start = middle+1;
		else
			end = middle;
	}
}

PendulumSimulation create_pendulum_simulation(const SimMonitorData& md)
{
	float hip_angle = hip_angle_0+scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
	float leg_angle = (1-md.common_leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
	float leg_angle_vel = -md.common_leg_pos_dot * (tau / leg_gear_reduction);
	
	PendulumSimulation p;
	p.gravity = earth_acc*1000;
	p.x1 = lower_leg_length-scd.sim_squat_lower_leg_cog_y;
	p.l1 = lower_leg_length;
	p.m1 = scd.sim_squat_lower_leg_mass*scd.sim_squat_body_mass_factor;
	p.I1 = scd.sim_squat_lower_leg_inertia*1000000*scd.sim_squat_body_inertia_factor;

	p.x2 = upper_leg_length-scd.sim_squat_upper_leg_cog_y;
	p.l2 = upper_leg_length;
	p.m2 = scd.sim_squat_upper_leg_mass*scd.sim_squat_body_mass_factor;
	p.I2 = scd.sim_squat_upper_leg_inertia*1000000*scd.sim_squat_body_inertia_factor;

	p.x3 = sqrt(sq(scd.sim_squat_body_cog_x) + sq(scd.sim_squat_body_cog_y));
	p.l3 = p.x3;
	p.m3 = scd.sim_squat_body_mass*scd.sim_squat_body_mass_factor;
	p.I3 = scd.sim_squat_body_inertia*1000000*scd.sim_squat_body_inertia_factor;

	p.theta1 = md.imu_x_angle_interframe-md.sim_target_x_angle_delta+hip_angle+leg_angle;
	p.theta2 = md.imu_x_angle_interframe-md.sim_target_x_angle_delta+hip_angle;
	p.theta3 = md.imu_x_angle_interframe-md.sim_target_x_angle_delta+atan2(scd.sim_squat_body_cog_x, -scd.sim_squat_body_cog_y);
			
	p.theta1_vel = md.theta_dot + md.common_hip_pos_sensor_dot + leg_angle_vel;
	p.theta2_vel = md.theta_dot + md.common_hip_pos_sensor_dot;
	p.theta3_vel = md.theta_dot;

	p.wheel_radius = wheel_radius;
	p.wheel_inertia = scd.sim_squat_wheel_inertia*1000000;
	p.wheel_mass = scd.sim_squat_wheel_mass;
	p.wheel_pos = md.foot_pos                  *tau + md.imu_x_angle_interframe + hip_angle                    + leg_angle    ;
	p.wheel_vel = md.foot_vel_coarse_interframe*tau + md.theta_dot              + md.common_hip_pos_sensor_dot + leg_angle_vel;
	
	p.rubber_pos = md.rubber_pos               *tau + md.imu_x_angle_interframe + hip_angle                    + leg_angle    ;
	p.rubber_vel = md.rubber_vel_interframe    *tau + md.theta_dot              + md.common_hip_pos_sensor_dot + leg_angle_vel;
	p.rubber_factor  = scd.sim_squat_rubber_factor*1000000;
	p.rubber_inertia = scd.sim_squat_rubber_inertia*1000000;
	return p;
}

void sim_update(SimMonitorData& md)
{
	const float delta_time = scd.target_delta_time_ms * 0.001f;

	md.common_hip_pos_target += md.common_servo_vel*delta_time;
	md.common_hip_pos_target = clamp(md.common_hip_pos_target, 0, 1);
	
	float hip_angle = hip_angle_0+scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
	float hip_angle_target = hip_angle_0+scd.hip_sensor_angle_range*md.common_hip_pos_target;
	float leg_angle = (1-md.common_leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
	float leg_angle_vel = -md.common_leg_pos_dot * (tau / leg_gear_reduction);
	
	PendulumSimulation p = create_pendulum_simulation(md);

	for (int i = 0; i < scd.num_sim_iterations; i++)
	{
		float foot_vel = (p.wheel_vel-(md.theta_dot + (p.theta2_vel-p.theta3_vel) + (p.theta1_vel-p.theta2_vel)))/tau;
		float vel_error = md.foot_vel_target - foot_vel;
		md.foot_integrator += vel_error * scd.foot_control_vel_integrator_gain * delta_time / scd.num_sim_iterations;
		md.foot_current = vel_error * scd.foot_control_vel_gain + md.foot_integrator;
		md.foot_current = clamp(md.foot_current, -scd.foot_control_current_lim, scd.foot_control_current_lim);

		const int num_foot_motors = 2;
		// 8.3 from formula here: https://things-in-motion.blogspot.com/2018/12/how-to-estimate-torque-of-bldc-pmsm.html
		float foot_torque = num_foot_motors*8.3f*md.foot_current / scd.foot_control_motor_kv;

		float common_leg_pos = 1-((p.theta1-p.theta2-leg_angle_1) / (tau / leg_gear_reduction));
		float leg_vel = -(p.theta1_vel-p.theta2_vel) / (tau / leg_gear_reduction);
		float leg_torque = 0;
		float hip_torque = 0;
		if (md.jump_phase == JP_Landing)
		{
			bool stop_landing = false;
#ifdef NEW_LANDING
			{
				float leg_angle = (1-common_leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
				float leg_angle_vel = -leg_vel * (tau / leg_gear_reduction);
				md.l_leg_height_pos = -cos(md.l_hip_angle + leg_angle);
				md.l_leg_height_vel = sin(md.l_hip_angle + leg_angle)*leg_angle_vel;
			}
#if 1
			float leg_vel2 = md.l_leg_height_vel;
			float leg_pos2 = md.l_leg_height_pos;
			float leg_pos_target2 = -cos(md.l_hip_angle + ((1-md.common_leg_pos_target) * (tau / leg_gear_reduction) + leg_angle_1));
#else
			float leg_vel2 = leg_vel;
			float leg_pos2 = common_leg_pos;
			float leg_pos_target2 = md.common_leg_pos_target;
#endif

			if (leg_vel2 > md.l_max_vel)
				md.l_max_vel = leg_vel2;
			float pos_err_0 = leg_pos_target2-leg_pos2;
			if (pos_err_0 < 0) stop_landing = true;
			md.l_max_vel = max(md.l_max_vel, 2*pos_err_0/scd.l_max_time);
			md.l_vel_target_delta -= 0.5f * sq(md.l_vel_target) / pos_err_0 * delta_time / scd.num_sim_iterations;
			md.l_vel_target = md.l_max_vel + md.l_vel_target_delta;
			if (md.l_vel_target < 0) stop_landing = true;
			//float sync_err = pos_err_0 + pos_err_1;
			//vel_des_0 += sync_err*cd.pos_err_sync_gain;
			//vel_des_1 += sync_err*cd.pos_err_sync_gain;
			float v_err_0 = md.l_vel_target - leg_vel2;
	        md.leg_current = scd.l_vel_gain * v_err_0;
#else
			float pos_err_0 = md.common_leg_pos_target-common_leg_pos;
			float vel_des_0 = pos_err_0*scd.l_pos_gain;
			//float sync_err = pos_err_0 + pos_err_1;
			//vel_des_0 += sync_err*cd.pos_err_sync_gain;
			//vel_des_1 += sync_err*cd.pos_err_sync_gain;
			vel_des_0 = clamp(vel_des_0, -scd.l_vel_limit, scd.l_vel_limit);
			float v_err_0 = vel_des_0 - leg_vel;
	        md.leg_current = scd.l_vel_gain * v_err_0;
#endif
			md.leg_current = clamp(md.leg_current, -scd.f_current_lim, scd.f_current_lim);
			leg_torque -= num_foot_motors*8.3f*md.leg_current * leg_gear_reduction / scd.leg_control_motor_kv;
			if (stop_landing)
			{
				// Set input_pos now while input_mode is still INPUT_MODE_PASSTHROUGH
				// Otherwise input_pos would slowly creep towards the current value
				//md.common_leg_pos_target = md.common_leg_pos;
				md.common_leg_pos_target = clamp(md.common_leg_pos_target, scd.stand_control_min_leg_angle, scd.stand_control_max_leg_angle);
				md.common_leg_pos_target_odrv = md.common_leg_pos_target;
				//set_abs_input_pos0(md.common_leg_pos_target);
				//set_abs_input_pos1(md.common_leg_pos_target);
				md.jump_phase = JP_None;
				leg_torque = 0;
			}
		}
		if (md.jump_phase != JP_Landing)
		{
			if (1)
			{
				// INPUT_MODE_POS_FILTER:
			    float bandwidth = scd.input_filter_bandwidth;
				float input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time
				float input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped

				float delta_pos = md.common_leg_pos_target - md.common_leg_pos_target_odrv; // Pos error
				float delta_vel = 0 - md.common_leg_pos_dot_target_odrv; // Vel error
				float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
				//torque_setpoint_ = accel * config_.inertia; // Accel
				md.common_leg_pos_dot_target_odrv += delta_time / scd.num_sim_iterations * accel; // delta vel
				md.common_leg_pos_target_odrv     += delta_time / scd.num_sim_iterations * md.common_leg_pos_dot_target_odrv; // Delta pos


				float pos_err = md.common_leg_pos_target_odrv-common_leg_pos;
				float vel_des = md.common_leg_pos_dot_target_odrv + pos_err*scd.pos_gain;
				vel_des = clamp(vel_des, -scd.vel_limit, scd.vel_limit);
				float v_err = vel_des - leg_vel;
				md.leg_current = scd.vel_gain * v_err + md.leg_integrator;
				md.leg_current = clamp(md.leg_current, -scd.current_lim, scd.current_lim);
				leg_torque -= num_foot_motors*8.3f*md.leg_current * leg_gear_reduction / scd.leg_control_motor_kv;

				md.leg_integrator += scd.vel_integrator_gain * delta_time / scd.num_sim_iterations * v_err;
			}
			else
			{
				leg_torque += (md.common_leg_pos_target - common_leg_pos) * scd.leg_motor_p_gain;
				leg_torque += (0                          - leg_vel)          * scd.leg_motor_d_gain;
			}
		}
		hip_torque += (hip_angle_target - (p.theta2-p.theta3+atan2(scd.sim_squat_body_cog_x, -scd.sim_squat_body_cog_y))) * scd.hip_motor_p_gain;
		hip_torque += (0 - (p.theta2_vel-p.theta3_vel))                                                           * scd.hip_motor_d_gain;

		p.update(delta_time / scd.num_sim_iterations, foot_torque*1000000, leg_torque*1000000, hip_torque*1000000);
		
		if (float(i) < scd.vel_sensor_delay * scd.num_sim_iterations)
			md.foot_vel_coarse = foot_vel;
		if (float(i) < scd.imu_sensor_delay * scd.num_sim_iterations)
			md.imu_x_angle = p.theta3 + md.sim_target_x_angle_delta - atan2(scd.sim_squat_body_cog_x, -scd.sim_squat_body_cog_y);
	}

	md.theta_acc = lerp((p.theta3_vel-md.theta_dot) / delta_time,
						md.theta_acc,
						scd.theta_acc_smooth);

	md.theta_dot = p.theta3_vel;

	md.common_hip_pos_sensor_dot = p.theta2_vel-p.theta3_vel;
	leg_angle_vel = p.theta1_vel-p.theta2_vel;
	md.common_leg_pos_dot = -leg_angle_vel / (tau / leg_gear_reduction);
		
	md.imu_x_angle_interframe = p.theta3 + md.sim_target_x_angle_delta - atan2(scd.sim_squat_body_cog_x, -scd.sim_squat_body_cog_y);

	md.common_hip_pos_sensor = (p.theta2-p.theta3+atan2(scd.sim_squat_body_cog_x, -scd.sim_squat_body_cog_y)-hip_angle_0)/scd.hip_sensor_angle_range;
	md.common_leg_pos = 1-((p.theta1-p.theta2-leg_angle_1) / (tau / leg_gear_reduction));

	hip_angle = hip_angle_0+scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
	leg_angle = (1-md.common_leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;

	md.foot_pos                   = (p.wheel_pos-(md.imu_x_angle_interframe + hip_angle                    + leg_angle    ))/tau;
	md.foot_vel_coarse_interframe = (p.wheel_vel-(md.theta_dot              + md.common_hip_pos_sensor_dot + leg_angle_vel))/tau;

	md.rubber_pos                   = (p.rubber_pos-(md.imu_x_angle_interframe + hip_angle                    + leg_angle    ))/tau;
	md.rubber_vel_interframe        = (p.rubber_vel-(md.theta_dot              + md.common_hip_pos_sensor_dot + leg_angle_vel))/tau;

	md.balance_control_pos_user += md.balance_control_vel_user*scd.control_vel_factor * delta_time;

	if (md.jump_phase != JP_Landing)
	{
		md.common_leg_pos_target += md.common_leg_vel_target*delta_time;
		md.common_leg_pos_target = clamp(md.common_leg_pos_target, scd.stand_control_min_leg_angle, scd.stand_control_max_leg_angle);
	}

	md.sim_target_x_angle_delta += md.fake_x_angle_delta_vel_user*scd.fake_x_angle_delta_vel * delta_time;
}

void sim_policy(SimMonitorData& md, float history_theta)
{
	const float delta_time = scd.target_delta_time_ms * 0.001f;

	md.post_landing_timer += delta_time;
	if (md.jump_phase == JP_Landing)
	{
		// Check if it is time to stop landing.
		// We stop, when either both velocities are negative
		// or both are low for some time.
		bool stop_landing = false;

#ifndef NEW_LANDING
		const float low_vel = 0.5f;
		float leg_vel = md.common_leg_pos_dot;
		if (leg_vel < low_vel)
			md.low_vel_timer += delta_time;
		else
			md.low_vel_timer = 0;
		if (md.low_vel_timer > 0.1f) stop_landing = true;
			
		const float low_vel2 = 0;
		if (leg_vel < low_vel2)
			md.low_vel2_timer += delta_time;
		else
			md.low_vel2_timer = 0;
		if (md.low_vel2_timer > 0.05f) stop_landing = true;

		//if (cd.force_stay_in_landing_mode) stop_landing = false;
#endif
			
		if (!md.is_standing) stop_landing = true;
		if (stop_landing)
		{
			md.low_vel_timer = 0;
			md.low_vel2_timer = 0;
			//printf("stop landing!\n");

			// Set input_pos now while input_mode is still INPUT_MODE_PASSTHROUGH
			// Otherwise input_pos would slowly creep towards the current value
			md.common_leg_pos_target = md.common_leg_pos;
			md.common_leg_pos_target = clamp(md.common_leg_pos_target, scd.stand_control_min_leg_angle, scd.stand_control_max_leg_angle);
			md.common_leg_pos_target_odrv = md.common_leg_pos_target;
			//set_abs_input_pos0(md.common_leg_pos_target);
			//set_abs_input_pos1(md.common_leg_pos_target);
			md.jump_phase = JP_None;
		}
	}


	float old_target_angle = md.balance_control_target_angle;
	md.balance_control_target_angle = calculate_squat_angle(md.common_hip_pos_sensor, md.common_leg_pos, &md.squat_length);

	md.balance_control_target_angle_vel = update_smooth(
		md.balance_control_target_angle_vel,
		(md.balance_control_target_angle-old_target_angle) / delta_time,
		scd.balance_control_target_angle_vel_smooth, scd.target_delta_time_ms);

	{
		float th_pos = history_theta;
		float leg_pos = md.common_leg_pos;
		float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
		float hip_angle = hip_angle_0 + scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
		float foot_pos  =  md.foot_pos;
		float coi_x, coi_z;
		get_com_world_position(md.imu_x_angle, th_pos,
				hip_angle, leg_angle, foot_pos,
				hip_angle, leg_angle, foot_pos,
				0, 0,
				&coi_x, &coi_z,
				&md.coi_theta_acc);
		//md.imu_sensor_to_floor_pos_x = coi_x;
		//md.imu_sensor_to_floor_pos_y = coi_z;
		
		float old_coi_world_pos = md.coi_world_pos;
		md.coi_world_pos = coi_x;

		float old_coi_world_vel = md.coi_world_vel;
		md.coi_world_vel = update_smooth(
				old_coi_world_vel,
				(md.coi_world_pos-old_coi_world_pos) / delta_time,
				cd.coi_world_vel_smooth_factor, scd.target_delta_time_ms);

		float old_coi_world_acc = md.coi_world_acc;
		md.coi_world_acc = update_smooth(
				old_coi_world_acc,
				(md.coi_world_vel-old_coi_world_vel) / delta_time / 1000 / earth_acc,
				cd.coi_world_acc_smooth_factor, scd.target_delta_time_ms);
	}

	float old_error_p = md.error_p;
	md.error_p = md.imu_x_angle - md.balance_control_target_angle - md.t_correction_angle;
	//md.avg_error_p = md.avg_error_p * (1-cd.stepper_avg_smooth) + md.error_p * cd.stepper_avg_smooth;

	md.error_d = update_smooth(
		md.error_d,
		(md.error_p-old_error_p) / delta_time,
		scd.balance_control_error_d_smooth, scd.target_delta_time_ms);
	
	{
		float new_smooth = update_smooth(
				md.balance_control_vel_quasi_smooth,
				//md.balance_control_vel_quasi,
				md.balance_control_vel_user*scd.control_vel_factor,
				scd.balance_control_vel_quasi_smooth_factor, scd.target_delta_time_ms);
		md.balance_control_vel_quasi_smooth = clamp(
				new_smooth,
				md.balance_control_vel_quasi_smooth-scd.balance_control_vel_quasi_smooth_max_acc*delta_time,
				md.balance_control_vel_quasi_smooth+scd.balance_control_vel_quasi_smooth_max_acc*delta_time);
	}

	//bool stop_motors = false;
	//if (md.jump_phase == JP_Jumping || md.jump_phase == JP_LegUp)
	//	stop_motors = true;
	//if (md.jump_phase == JP_Falling && !md.enabled_landing_detector)
	//	stop_motors = true;
	//if (!stop_motors || md.balance_control_force_still)
	{
		float balance_control_gain_p               = use_control_gains ? scd.balance_control_gain_p               : md.balance_control_gain_p;
		float balance_control_gain_i               = use_control_gains ? scd.balance_control_gain_i               : md.balance_control_gain_i;
		float balance_control_gain_d               = use_control_gains ? scd.balance_control_gain_d               : md.balance_control_gain_d;
		float balance_control_gain_vel_high        = use_control_gains ? scd.balance_control_gain_vel_high        : md.balance_control_gain_vel_high;
		float balance_control_gain_vel_low         = use_control_gains ? scd.balance_control_gain_vel_low         : md.balance_control_gain_vel_low;
		float balance_control_gain_backcalculation = use_control_gains ? scd.balance_control_gain_backcalculation : md.balance_control_gain_backcalculation;
	
		md.vel_p = md.error_p * balance_control_gain_p;
		md.vel_d = md.error_d * balance_control_gain_d;
		{
			// We clamp theta that gets added to integrator.
			// This stabilizes recovery in extreme balance situations.
			float theta = md.error_p;

			theta = clamp(theta, -scd.balance_control_max_theta_integrator, scd.balance_control_max_theta_integrator);
			md.balance_control_integrator += theta * delta_time * balance_control_gain_i;
		}

		md.balance_control_position_control_error      = scd.balance_control_position_control_target - md.coi_world_pos/(tau*wheel_radius);
		float balance_control_position_control_error_d = 0                                           - md.coi_world_vel/(tau*wheel_radius);

		md.vel_pos = 0;
		{
			md.vel_pos += clamp(scd.balance_control_use_coi_vel ? md.coi_world_vel/(tau*wheel_radius) : md.balance_control_vel_output, -scd.foot_control_vel_limit, scd.foot_control_vel_limit);
			if (md.balance_control_enable_position_control)
			{
				float balance_control_position_control_gain_p = clamp_and_map(md.squat_length, 80, 160, scd.balance_control_position_control_gain_p_low, scd.balance_control_position_control_gain_p_high);
				float balance_control_position_control_gain_d = clamp_and_map(md.squat_length, 80, 160, scd.balance_control_position_control_gain_d_low, scd.balance_control_position_control_gain_d_high);

				float target_vel = md.balance_control_position_control_error  *balance_control_position_control_gain_p;
				target_vel      +=    balance_control_position_control_error_d*balance_control_position_control_gain_d;
				target_vel = clamp(target_vel, -scd.balance_control_position_control_max_vel, scd.balance_control_position_control_max_vel);
				md.vel_pos -= target_vel;
			}
			else
			{
				//md.vel_pos -= md.balance_control_vel_quasi;
				md.vel_pos -= md.balance_control_vel_user*scd.control_vel_factor;
			}
			float balance_control_gain_vel = balance_control_gain_vel_high;
			if (balance_control_gain_vel_low)
				balance_control_gain_vel = clamp_and_map(md.squat_length, 80, 160, balance_control_gain_vel_low, balance_control_gain_vel_high);
			if (md.jump_phase != JP_Positioning && scd.balance_control_gain_vel_landing_warmup_time != 0)
				balance_control_gain_vel = lerp(0, balance_control_gain_vel, std::min(md.post_landing_timer / scd.balance_control_gain_vel_landing_warmup_time, 1.0f));
			md.vel_pos *= balance_control_gain_vel * delta_time;

			// Clamping this error effectivly limits the acceleration of target velocity.
			// This is important in case balance_control_vel_quasi changes rapidly.
			md.vel_pos = clamp(md.vel_pos, -scd.balance_control_vel_acceleration_limit, scd.balance_control_vel_acceleration_limit);

			md.balance_control_integrator += md.vel_pos;
		}

		// backcalculation
		float vel_error = md.foot_vel_coarse-(md.balance_control_vel_output+md.balance_control_delta_vel /*-md.balance_control_rotation_error*cd.balance_control_rotation_error_factor*/);
		//float vel_error_r = -md.foot_vel_coarse[1]-(md.balance_control_vel_output+md.balance_control_delta_vel_r+md.balance_control_rotation_error*cd.balance_control_rotation_error_factor);
		md.vel_backcalculation = vel_error * delta_time * balance_control_gain_backcalculation;
		md.balance_control_integrator += md.vel_backcalculation;

		// Tiny gain that improves stability during hip or leg movement
		md.balance_control_integrator += md.balance_control_target_angle_vel*scd.balance_control_target_angle_vel_gain;
		
		md.balance_control_integrator = clamp(md.balance_control_integrator, -scd.max_balance_control_integrator, scd.max_balance_control_integrator);

		// hip balance integrator (unused currently)
		//md.balance_control_integrator += md.hip_balance_current * cd.hip_balance_integrator;
		md.balance_control_vel_output = md.vel_p + md.balance_control_integrator + md.vel_d;

		if (md.balance_control_force_still)
		{
			md.balance_control_vel_output = 0;
			//md.post_landing_timer = 0; // uncomment this to test balancing with extreme theta
			md.balance_control_integrator = 0;
		}
#ifdef NEW_LANDING
		else if (md.jump_phase == JP_LegUp || md.jump_phase == JP_Falling || md.post_landing_timer < scd.landing_fixed_foot_vel_time)
		{
			// For a short time after landing, we set a fixed minimum forward speed.
			// This makes it easier for the leg to close and has the effect that the
			// fall energy is absorbed in the leg motors. Otherwise the body of the robot
			// starts to rotate and we run into balancing problems.
			md.balance_control_vel_output -= md.balance_control_integrator;
			md.balance_control_integrator = scd.landing_fixed_foot_vel+md.balance_control_vel_user;
			md.balance_control_vel_output += md.balance_control_integrator;
		}
#else
		else if ((md.jump_phase == JP_LegUp || md.jump_phase == JP_Falling || md.post_landing_timer < scd.landing_fixed_foot_vel_time) &&
				md.error_p > 0.02f)
		{
			// For a short time after landing, we set a fixed minimum forward speed.
			// This makes it easier for the leg to close and has the effect that the
			// fall energy is absorbed in the leg motors. Otherwise the body of the robot
			// starts to rotate and we run into balancing problems.
			md.balance_control_vel_output = std::max(md.balance_control_vel_output, scd.landing_fixed_foot_vel);
			md.balance_control_integrator = 0;
		}
#endif
		//else if (abs(md.balance_control_rotation_vel) < cd.t_correction_max_rotation_vel)
		else
		{
			//float t_correction_error = md.balance_control_vel_output-md.balance_control_vel_quasi;
			float t_correction_error = md.balance_control_vel_output-md.balance_control_vel_user*scd.control_vel_factor;
			md.t_correction_angle -= sign(t_correction_error) * cd.t_correction_simple_factor * delta_time;
			
			md.t_correction_angle_raw += (md.coi_theta_acc-md.coi_world_acc)*cd.t_correction_acc_p*delta_time;

			if (cd.do_t_correction_acc)
			{
				md.t_correction_angle = update_smooth(
						md.t_correction_angle,
						md.t_correction_angle_raw,
						cd.t_correction_acc_smooth, cd.target_delta_time_ms);
			}

			if (md.t_correction_angle < -cd.t_correction_angle_max) { md.t_correction_angle = -cd.t_correction_angle_max; }
			if (md.t_correction_angle > cd.t_correction_angle_max ) { md.t_correction_angle =  cd.t_correction_angle_max; }
		}


		/*if (md.startup_sequence == SS_StartBalancing2)
		{
			md.is_standing = abs(md.error_p) < cd.startup_balance_control_standing_tolerance;
			if (abs(md.error_p) < cd.balance_control_standing_theta_tolerance * 0.1f)
				md.startup_sequence = SS_FinalLegHipMovement;
		}
		else*/
		if (md.is_standing)
		{
			if (abs(md.error_p) < scd.balance_control_standing_theta_tolerance)
				md.balance_control_standing_action_tolerance_timer = 0;
			float delta = abs(md.balance_control_vel_output-md.balance_control_vel_quasi_smooth);
			if (delta > scd.balance_control_standing_action_tolerance)
			{
				md.balance_control_standing_action_tolerance_timer +=
						(delta - scd.balance_control_standing_action_tolerance) * delta_time;
				if (md.balance_control_standing_action_tolerance_timer > scd.balance_control_standing_action_tolerance_time)
				{
					//printf("action OOB: %f %f\n", md.balance_control_vel_output, md.balance_control_vel_quasi_smooth);
					md.is_standing = false;
				}
			}
			else
				md.balance_control_standing_action_tolerance_timer = 0;
		}
		else
		{
			md.is_standing = abs(md.error_p) < scd.balance_control_standing_theta_tolerance * 0.1f;
			//md.is_standing = abs(md.error_p) < cd.stepper_standing_tolerance;
		}
			
	}
	/*else
	{
		// We are in falling mode, we keep the motors fixed.
		md.balance_control_vel_output = 0;
		//md.balance_control_pos_output = 0;
		//md.balance_control_pos_user = 0;
		//md.balance_control_pos_quasi = 0;
		md.is_standing = true;
		md.vel_p = 0;
		md.balance_control_integrator = 0;
		md.vel_pos = 0;
		//md.t_correction_angle = 0;
	}*/

	/*if (!md.is_standing)
	{
		md.vel_p = 0;
		md.vel_d = 0;
		md.balance_control_integrator = 0;
		md.vel_pos = 0;
		md.t_correction_angle = 0;
		//md.balance_control_pos_output = 0;
		md.balance_control_vel_output = 0;
		//md.balance_control_pos_user = 0;
		//md.balance_control_vel_user = 0;
		//md.balance_control_pos_quasi = 0;
		//md.balance_control_vel_quasi = 0;
		//md.balance_control_vel_quasi_smooth = 0;

		//md.balance_control_rotation_error_bias = -md.foot_pos[0] - md.foot_pos[1];
	}*/

	{
		float leg_pos_l = md.common_leg_pos;
		//float leg_pos_r = md.common_leg_pos;
		float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
		//float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;
		float hip_angle_l = hip_angle_0 + scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
		//float hip_angle_r = hip_angle_0 + scd.hip_sensor_angle_range*md.common_servo_sensor;

		// If the angle between the lower leg and the gravity vector changes, we need to compensate
		// the resulting rotation:
		float delta_pos_lower_leg_angle_l = -(md.imu_x_angle + hip_angle_l + leg_angle_l) / tau;
		//float delta_pos_lower_leg_angle_r = -(md.imu_x_angle + hip_angle_r + leg_angle_r) / tau;

		// Also, when one foot is moved in front of the other, we need to rotate the corresponding wheel
		// the same amount in that direction. Otherwise the robot would start to turn.
		// This is especially important when the robot moves with one leg over a ramp.
		/*float delta_l = leg_delta_from_angles(md.imu_x_angle, hip_angle_l, leg_angle_l);
		float delta_r = leg_delta_from_angles(md.imu_x_angle, hip_angle_r, leg_angle_r);
		float delta = delta_l-delta_r;
		float delta_pos_foot_pos_l =  delta*0.5f/(tau*wheel_radius);
		float delta_pos_foot_pos_r = -delta*0.5f/(tau*wheel_radius);*/

		// Combine both corrections and calculate velocity
		float old_balance_control_delta_pos = md.balance_control_delta_pos;
		md.balance_control_delta_pos = delta_pos_lower_leg_angle_l;// + delta_pos_foot_pos_l;
		md.balance_control_delta_vel = (md.balance_control_delta_pos - old_balance_control_delta_pos) / delta_time;

		/*md.balance_control_rotation_error_bias += md.balance_control_rotation_vel * 2 * delta_time;
		md.balance_control_rotation_error = 0;
		md.balance_control_rotation_error += md.foot_pos[0];
		md.balance_control_rotation_error += md.foot_pos[1]; // (double negated)
		md.balance_control_rotation_error += md.balance_control_rotation_error_bias;
		md.balance_control_rotation_error -= md.balance_control_delta_pos_l;
		md.balance_control_rotation_error += md.balance_control_delta_pos_r;*/
	}
	//md.foot_vel_coarse = md.balance_control_vel_output;
	//md.foot_vel_target = md.balance_control_vel_output;

	// foot control
	float input_vel = 0;
	input_vel = md.balance_control_vel_output;
	//input_vel += md.balance_control_rotation_vel * (axis_no == 1 ? 1 : -1);
	input_vel += md.balance_control_delta_vel;
	//input_vel += md.balance_control_rotation_error * cd.balance_control_rotation_error_factor * (axis_no == 1 ? 1 : -1);
	//if (axis_no == 1)
	//	input_vel = -input_vel;
	//if (md.startup_sequence == SS_FootIndex2 && md.foot_motor_ready[axis_no])
	//	input_vel = 0.5f * (axis_no == 1 ? 1 : -1);
	
	//if (!should_run)
	//	input_vel = 0;

	// We limit the acceleration, otherwise it can get so big that
	// backcalculation in balance_control doesn't really work anymore when
	// the max_current is 20A or bigger.
	// Not sure why this is the case, maybe the mainloop is too slow.
	input_vel = md.foot_vel_target + clamp(input_vel-md.foot_vel_target,
			-scd.foot_control_accel_limit*delta_time,
			 scd.foot_control_accel_limit*delta_time);

	input_vel = clamp(input_vel, -scd.foot_control_vel_limit, scd.foot_control_vel_limit);
	md.foot_vel_target = input_vel;
}

static void draw_wheel(float r, int num_segments)
{
    glBegin(GL_LINE_STRIP);
    for(int ii = 0; ii < num_segments; ii++)
    {
        float theta = tau * float(ii) / float(num_segments);//get the current angle

        float x = r * cosf(theta);//calculate the x component
        float y = r * sinf(theta);//calculate the y component

        glVertex2f(x, y);//output vertex

    }
    glVertex2f(r, 0);
    glVertex2f(0, 0);
    glEnd();
}

static void draw_robot_leg(float x_angle, float hip_angle, float leg_angle, const SimMonitorData& md)
{
	float joint_x, joint_y;
	{
		// upper leg
		float x0 = -32.28f*0.5f, x1 = 32.28f*0.5f, y0 = 0, y1 = upper_leg_length;

		glPushMatrix();
		glRotatef(to_degree(x_angle + hip_angle), 0, 0, 1);
		
		glBegin(GL_LINE_LOOP);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();
		glPopMatrix();

		joint_x = -sin(x_angle + hip_angle)*y1;
		joint_y = cos(x_angle + hip_angle)*y1;
	}
		
	{
		// lower leg
		float x0 = -17.5f, x1 = 17.5f, y0 = 0, y1 = lower_leg_length;

		glPushMatrix();
		glTranslatef(joint_x, joint_y, 0);
		glRotatef(to_degree(x_angle + hip_angle + leg_angle), 0, 0, 1);
		
		glBegin(GL_LINE_LOOP);
		glVertex2f(x0, y1);
		glVertex2f(x1, y1);
		glVertex2f(x1, y0);
		glVertex2f(x0, y0);
		glEnd();
		glPopMatrix();

		joint_x += -sin(x_angle + hip_angle + leg_angle)*y1;
		joint_y +=  cos(x_angle + hip_angle + leg_angle)*y1;
	}

	{
		// wheel
		glPushMatrix();
		glTranslatef(joint_x, joint_y, 0);
		glRotatef(to_degree(x_angle + hip_angle + leg_angle + (md.foot_pos * tau)), 0, 0, 1);
		
		draw_wheel(wheel_radius*0.7f, 8);
		glPopMatrix();

		glPushMatrix();
		glTranslatef(joint_x, joint_y, 0);
		glRotatef(to_degree(x_angle + hip_angle + leg_angle + (md.rubber_pos * tau)), 0, 0, 1);
		
		draw_wheel(wheel_radius, 8);
		glPopMatrix();
	}
}


bool body_acc_update(SimMonitorData& md)
{
	const float delta_time = scd.target_delta_time_ms * 0.001f;

	float leg_pos = md.common_leg_pos;
	float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
	float hip_angle = hip_angle_0 + scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
	float foot_pos  =  md.foot_pos;

	// First we calculate the position of the imu sensor, relative to a fixed point on the floor.
	float imu_x, imu_y;
	get_imu_sensor_position(md.imu_x_angle,
			hip_angle, leg_angle, foot_pos,
			hip_angle, leg_angle, foot_pos,
			imu_x, imu_y);

	float old_imu_sensor_to_floor_pos_x = md.imu_sensor_to_floor_pos_x;
	float old_imu_sensor_to_floor_pos_y = md.imu_sensor_to_floor_pos_y;
	md.imu_sensor_to_floor_pos_x = imu_x;
	md.imu_sensor_to_floor_pos_y = imu_y;

	// Then we differentiate the position two times to get the acceleration.
	float old_imu_sensor_to_floor_vel_x = md.imu_sensor_to_floor_vel_x;
	float old_imu_sensor_to_floor_vel_y = md.imu_sensor_to_floor_vel_y;
	md.imu_sensor_to_floor_vel_x = update_smooth(
			old_imu_sensor_to_floor_vel_x,
			(md.imu_sensor_to_floor_pos_x-old_imu_sensor_to_floor_pos_x) / delta_time,
			scd.imu_sensor_to_floor_vel_smooth_factor, scd.target_delta_time_ms);
	md.imu_sensor_to_floor_vel_y = update_smooth(
			old_imu_sensor_to_floor_vel_y,
			(md.imu_sensor_to_floor_pos_y-old_imu_sensor_to_floor_pos_y) / delta_time,
			scd.imu_sensor_to_floor_vel_smooth_factor, scd.target_delta_time_ms);

	float old_imu_sensor_to_floor_acc_x = md.imu_sensor_to_floor_acc_x;
	float old_imu_sensor_to_floor_acc_y = md.imu_sensor_to_floor_acc_y;
	md.imu_sensor_to_floor_acc_x = update_smooth(
			old_imu_sensor_to_floor_acc_x,
			(md.imu_sensor_to_floor_vel_x-old_imu_sensor_to_floor_vel_x) / delta_time / 1000 / earth_acc,
			scd.imu_sensor_to_floor_acc_smooth_factor, scd.target_delta_time_ms);
	md.imu_sensor_to_floor_acc_y = update_smooth(
			old_imu_sensor_to_floor_acc_y,
			(md.imu_sensor_to_floor_vel_y-old_imu_sensor_to_floor_vel_y) / delta_time / 1000 / earth_acc,
			scd.imu_sensor_to_floor_acc_smooth_factor, scd.target_delta_time_ms);

	return true;
}

void draw_robot(float ui_x_pos, float ui_y_pos,
		float x_angle,
		float l_hip_angle, float l_leg_angle,
		float r_hip_angle, float r_leg_angle,
		float foot_pos_visual_reference, float balance_control_pos_user,
		float imu_x, float imu_y,
		const SimMonitorData& md)
{	
	float l_wheel_x, l_wheel_y;
	float r_wheel_x, r_wheel_y;
	hip_wheel_vector_from_angles(x_angle, l_hip_angle, l_leg_angle, l_wheel_x, l_wheel_y);
	hip_wheel_vector_from_angles(x_angle, r_hip_angle, r_leg_angle, r_wheel_x, r_wheel_y);

	float delta_x = (l_wheel_x + r_wheel_x)*0.5f;
	float delta_y = -max(l_wheel_y, r_wheel_y); 

	delta_x += md.rubber_pos*tau*wheel_radius;
	delta_x += foot_pos_visual_reference*tau*wheel_radius;

	delta_x -= balance_control_pos_user * tau * wheel_radius;

	delta_x += (x_angle + l_hip_angle + l_leg_angle)*wheel_radius*0.5f;
	delta_x += (x_angle + r_hip_angle + r_leg_angle)*wheel_radius*0.5f;
	
	// draw robot from the side
	{
		glPushMatrix();
		glTranslatef(ui_x_pos, ui_y_pos, 0);
		glScalef(-0.5f*dpi_scaling, 0.5f*dpi_scaling, 1);
		
		{
			// floor
			glBegin(GL_LINE_STRIP);
			glColor3f(0, 0, 0);
			glVertex2f(-500, wheel_radius);
			{
				const float step_size = 150;
				float floor_delta = (foot_pos_visual_reference-balance_control_pos_user)*tau*wheel_radius/step_size;
				float offset = floor_delta - floor(floor_delta);
				for (int i = 0; ; i++)
				{
					float x = -500+offset*step_size+i*step_size;
					if (x >= 500)
						break;
					glVertex2f(x, wheel_radius);
					glVertex2f(x, wheel_radius+10);
					glVertex2f(x, wheel_radius);
				}
			}
			glVertex2f(500, wheel_radius);
			glEnd();
		}

		{
			// imu position
			imu_x += (foot_pos_visual_reference-balance_control_pos_user) * tau * wheel_radius;
			float x0 = imu_x-5, x1 = imu_x+5, y0 = imu_y-5, y1 = imu_y+5;
			glPushMatrix();
			glBegin(GL_LINE_LOOP);
			glColor3f(0, 0, 0);
			glVertex2f(x0, y1);
			glVertex2f(x1, y1);
			glVertex2f(x1, y0);
			glVertex2f(x0, y0);
			glEnd();
			glPopMatrix();
		}

		glTranslatef(delta_x, delta_y, 0);
		
		{
			// robot body dimensions in mm
			float x0 = -31, x1 = 31, y0 = 12.5, y1 = -169;
			glPushMatrix();
			glRotatef(to_degree(x_angle), 0, 0, 1);
		
			glBegin(GL_LINE_LOOP);
			glColor3f(0, 0, 0);
			glVertex2f(x0, y1);
			glVertex2f(x1, y1);
			glVertex2f(x1, y0);
			glVertex2f(x0, y0);
			glEnd();
			glPopMatrix();
		}

		glColor3f(0.7f, 0.7f, 0.7f);
		draw_robot_leg(x_angle, l_hip_angle, l_leg_angle, md);
		glColor3f(0, 0, 0);
		draw_robot_leg(x_angle, r_hip_angle, r_leg_angle, md);

		glPopMatrix();
	}

	// squat cog side view 3
	{
		glPushMatrix();
		glTranslatef(ui_x_pos, ui_y_pos, 0);
		glScalef(-0.5f*dpi_scaling, 0.5f*dpi_scaling, 1);

		{
			float hip_angle = hip_angle_0+scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
			float leg_pos = md.common_leg_pos;
			float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;

			PendulumSimulation p = create_pendulum_simulation(md);

			float px0 = p.rubber_pos*wheel_radius;
			px0 += foot_pos_visual_reference*tau*wheel_radius;
			px0 -= balance_control_pos_user * tau * wheel_radius;

			float py0 = 0;

			float px1 = px0 + sin(p.theta1)*p.x1;
			float py1 = py0 - cos(p.theta1)*p.x1;

			float px2 = px0 + sin(p.theta1)*p.l1;
			float py2 = py0 - cos(p.theta1)*p.l1;

			float px3 = px2 + sin(p.theta2)*p.x2;
			float py3 = py2 - cos(p.theta2)*p.x2;

			float px4 = px2 + sin(p.theta2)*p.l2;
			float py4 = py2 - cos(p.theta2)*p.l2;

			float px5 = px4 + sin(p.theta3)*p.x3;
			float py5 = py4 - cos(p.theta3)*p.x3;

			float cog_x = (px5 * cd.squat_body_mass + px3 * cd.squat_upper_leg_mass + px1 * cd.squat_lower_leg_mass) / (cd.squat_body_mass + cd.squat_upper_leg_mass + cd.squat_lower_leg_mass);
			float cog_y = (py5 * cd.squat_body_mass + py3 * cd.squat_upper_leg_mass + py1 * cd.squat_lower_leg_mass) / (cd.squat_body_mass + cd.squat_upper_leg_mass + cd.squat_lower_leg_mass);

			glBegin(GL_LINE_STRIP);
			glColor3f(1, 0, 0);
			glVertex2f(px0, py0);
			glVertex2f(px1, py1);
			glVertex2f(px2, py2);
			glVertex2f(px3, py3);
			glVertex2f(px4, py4);
			glVertex2f(px5, py5);
			glEnd();

			glPointSize(3);
			glBegin(GL_POINTS);
			glVertex2f(px1, py1);
			glVertex2f(px3, py3);
			glVertex2f(px5, py5);
			glEnd();

			glColor3f(0, 0, 1);
			glPointSize(3);
			glBegin(GL_POINTS);
			glVertex2f(cog_x, cog_y);
			glEnd();
		}
		glPopMatrix();
	}
}



#define PLOT_HISTORY(name, var) \
	{ \
		static bool show_s = false; \
		static bool show_r = false; \
		plot_history(main_plot, real ? "real " name : "sim " name, [real](s64 i) \
			{ \
				SimMonitorData& md = real ? history[i].rmd : history[i].smd; \
				return var; \
			}, real ? &show_r : &show_s); \
	}

bool simulate_world = false;
bool simulate_policy = false;
bool do_interactive = false;
int simulation_warmup_steps = 0;
const int simulation_max_steps = 3000;
float initial_sim_theta = 0;
float initial_sim_theta_dot = 0;
float initial_foot_integrator = 0;
float initial_balance_control_integrator = 0;

#if 1
// "monitor_log_table_fail2.bin"
// history_plot_pos = 196; // Normal jump
int simulate_landing_steps = 41*0;
float landing_common_leg_pos = 0.47f;
float landing_common_leg_pos_dot = 8;
float landing_imu_x_angle = -0.04f;
float landing_imu_x_angle_dot = 6.46f;
float landing_foot_vel = 1.96f;
float landing_balance_control_integrator = 0;
#else
// "monitor_log_table_fail2.bin"
// history_plot_pos = 441; // Table jump (fail)
int simulate_landing_steps = 56;
float landing_common_leg_pos = 0.68f;
float landing_common_leg_pos_dot = 2;
float landing_imu_x_angle = 0.44f;
float landing_imu_x_angle_dot = 1.9f;
float landing_foot_vel = 5.8f;
float landing_balance_control_integrator = 0;
#endif

void run_simulation()
{
	s64 start, end;
	plot_get_display_range(main_plot, &start, &end);

	s64 position = start+simulation_warmup_steps;
	for (s64 i = start-50; i <= position; i++)
	{
		if (i < 0) continue;
		SimRealData& srd = history[i];
		srd.smd = srd.rmd;
	}
	SimRealData& srd = history[position];
	srd.smd.sim_target_x_angle_delta = srd.smd.t_correction_angle-cd.squat_base_angle2+scd.sim_target_x_angle_delta;
	if (initial_sim_theta || initial_sim_theta_dot)
	{
		srd.smd.imu_x_angle = initial_sim_theta;
		srd.smd.imu_x_angle_interframe = initial_sim_theta;
		srd.smd.theta_dot = initial_sim_theta_dot;
		srd.smd.foot_integrator = initial_foot_integrator;
		if (initial_balance_control_integrator)
			srd.smd.balance_control_integrator = initial_balance_control_integrator;
	}

	if (end-position > simulation_max_steps)
		end = position + simulation_max_steps;

	if (simulate_world || simulate_policy)
		for (s64 i = position+1; i < end; i++)
		{
			history[i].smd = history[i-1].smd;
			if (simulate_landing_steps != 0 && i-position == simulate_landing_steps)
			{
#ifdef NEW_LANDING
				history[i].smd.l_hip_angle =
						scd.l_base_angle_delta +
						history[i].smd.imu_x_angle +
						hip_angle_0+scd.hip_sensor_angle_range*history[i].smd.common_hip_pos_sensor;
#endif

				history[i].smd.common_leg_pos            = landing_common_leg_pos;
				history[i].smd.common_leg_pos_target     = landing_common_leg_pos;
				history[i].smd.common_leg_pos_target_odrv= landing_common_leg_pos;
				history[i].smd.common_leg_pos_dot        = landing_common_leg_pos_dot;
				history[i].smd.imu_x_angle_interframe      = landing_imu_x_angle;
				history[i].smd.imu_x_angle                 = landing_imu_x_angle;
				history[i].smd.theta_dot                   += landing_imu_x_angle_dot;
				float target_angle = calculate_squat_angle(history[i].smd.common_hip_pos_sensor, history[i].smd.common_leg_pos);
				history[i].smd.balance_control_target_angle = target_angle;
				history[i].smd.error_p = history[i].smd.imu_x_angle - target_angle - history[i].smd.t_correction_angle;
				float foot_vel = landing_foot_vel;
#ifdef NEW_LANDING
				//foot_vel = history[i].smd.error_p * scd.balance_control_gain_p + scd.landing_fixed_foot_vel + history[i].smd.balance_control_vel_user;
#endif
				history[i].smd.foot_vel_coarse_interframe  = foot_vel;
				history[i].smd.rubber_vel_interframe       = foot_vel;
				history[i].smd.foot_vel_coarse             = foot_vel;
				history[i].smd.foot_vel_target             = foot_vel;
				history[i].smd.balance_control_vel_output  = foot_vel;
				history[i].smd.balance_control_integrator = landing_balance_control_integrator;
				history[i].smd.l_max_vel = 0;
				history[i].smd.l_vel_target_delta = 0;

				float leg_pos_l = landing_common_leg_pos;
				float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
				float hip_angle_l = hip_angle_0 + scd.hip_sensor_angle_range*history[i].smd.common_hip_pos_sensor;
				history[i].smd.balance_control_delta_pos = -(history[i].smd.imu_x_angle + hip_angle_l + leg_angle_l) / tau;

				history[i].smd.post_landing_timer = 0;
				history[i].smd.jump_phase = JP_Landing;
		        history[i].smd.common_leg_pos_target = std::min(landing_common_leg_pos + scd.l_pos_zero_delta, scd.l_pos_zero_max);

				body_acc_update(history[i].smd);
			}
			body_acc_update(history[i].smd);

			history[i].smd.display_time                         = history[i].rmd.display_time;
			history[i].smd.balance_control_vel_user             = history[i].rmd.balance_control_vel_user;
			history[i].smd.fake_x_angle_delta_vel_user          = history[i].rmd.fake_x_angle_delta_vel_user;
			history[i].smd.common_leg_vel_target                = history[i].rmd.common_leg_vel_target;
			history[i].smd.common_servo_vel                     = history[i].rmd.common_servo_vel;
			//history[i].smd.common_leg_pos_target              = history[i].rmd.common_leg_pos_target;//
			//history[i].smd.common_hip_pos_target                  = history[i].rmd.common_hip_pos_target;
			history[i].smd.balance_control_force_still          = history[i].rmd.balance_control_force_still;
			history[i].smd.balance_control_gain_p				= history[i].rmd.balance_control_gain_p;
			history[i].smd.balance_control_gain_i				= history[i].rmd.balance_control_gain_i;
			history[i].smd.balance_control_gain_d				= history[i].rmd.balance_control_gain_d;
			history[i].smd.balance_control_gain_vel_high		= history[i].rmd.balance_control_gain_vel_high;
			history[i].smd.balance_control_gain_vel_low			= history[i].rmd.balance_control_gain_vel_low;
			history[i].smd.balance_control_gain_backcalculation	= history[i].rmd.balance_control_gain_backcalculation;
			history[i].smd.balance_control_enable_position_control = i-position > scd.balance_control_position_control_after_frame;
			if (simulate_world)
			{
				sim_update(history[i].smd);
			}
			else
			{
				history[i].smd.imu_x_angle              = history[i].rmd.imu_x_angle;
				history[i].smd.foot_vel_coarse          = history[i].rmd.foot_vel_coarse;
				history[i].smd.foot_pos                 = history[i].rmd.foot_pos;
				history[i].smd.rubber_pos               = history[i].rmd.rubber_pos;
				history[i].smd.balance_control_pos_user = history[i].rmd.balance_control_pos_user;
				history[i].smd.common_leg_pos_target  = history[i].rmd.common_leg_pos_target;
				history[i].smd.common_leg_pos         = history[i].rmd.common_leg_pos;
				history[i].smd.common_hip_pos_target    = history[i].rmd.common_hip_pos_target;
				history[i].smd.common_hip_pos_sensor    = history[i].rmd.common_hip_pos_sensor;
			}

			if (simulate_policy)
			{
				SimMonitorData& history_md = history[max(i-cd.coi_theta_past-1, (s64)0)].smd;
				float history_theta = history_md.error_p + history_md.t_correction_angle - history_md.t_correction_angle_raw;
				sim_policy(history[i].smd, history_theta);
			}
			else
			{
				history[i].smd.balance_control_vel_output = history[i].rmd.balance_control_vel_output;
				history[i].smd.foot_vel_target            = history[i].rmd.foot_vel_target;
				history[i].smd.vel_pos                    = history[i].rmd.vel_pos;
				history[i].smd.balance_control_integrator                  = history[i].rmd.balance_control_integrator;
				history[i].smd.vel_p                      = history[i].rmd.vel_p;
				history[i].smd.vel_d                      = history[i].rmd.vel_d;
				history[i].smd.vel_backcalculation        = history[i].rmd.vel_backcalculation;
				history[i].smd.t_correction_angle         = history[i].rmd.t_correction_angle;

				float target_angle = calculate_squat_angle(history[i].smd.common_hip_pos_sensor, history[i].smd.common_leg_pos);
				history[i].smd.error_p = history[i].smd.imu_x_angle - target_angle - history[i].smd.t_correction_angle;
			}
		}
}

void do_interactive_keys(SimMonitorData& md, bool* pressing_key)
{
	if (pressing_key[GLFW_KEY_UP])        md.balance_control_vel_user = 1;
	else if (pressing_key[GLFW_KEY_DOWN]) md.balance_control_vel_user = -1;
	else                                  md.balance_control_vel_user = 0;

	if (pressing_key[GLFW_KEY_LEFT])       md.fake_x_angle_delta_vel_user = 1;
	else if (pressing_key[GLFW_KEY_RIGHT]) md.fake_x_angle_delta_vel_user = -1;
	else                                   md.fake_x_angle_delta_vel_user = 0;
	
	md.common_leg_vel_target = 0;
	if (pressing_key['P']) md.common_leg_vel_target += -scd.common_leg_vel_user_max;
	if (pressing_key['L']) md.common_leg_vel_target +=  scd.common_leg_vel_user_max;

	md.common_servo_vel = 0;
	if (pressing_key['O']) md.common_servo_vel -= scd.common_servo_vel_user_max;
	if (pressing_key['K']) md.common_servo_vel += scd.common_servo_vel_user_max;
}

bool optimize_only_initial = true;
bool optimize_with_delta = false;
float vel_coarse_error_factor = 0.01f;
template<typename T>
void each_world_model_var(T t)
{
	t(initial_sim_theta);
	t(initial_sim_theta_dot);
	t(initial_foot_integrator);
	if (optimize_only_initial)
		return;
}

float sim_model_error(int start, int& end_out, int optimizer_max_steps)
{
	int end = end_out;
	if (!end)
		end = start+optimizer_max_steps;
	if (end <= start)
		return 0;

	float delta = 0;
	if (optimize_with_delta)
	{
		for (int i = start; i < end; i++)
		{
			delta += history[i].rmd.error_p - history[i].smd.error_p;
		}
		delta /= end-start;
	}

	float error = 0;
	for (int i = start; i < end; i++)
	{
		/*if (end_out == 0 && abs(history[i].rmd.error_p - history[i].smd.error_p) > 0.5f)
		{
			end_out = i;
			break;
		}*/
		error += sq(history[i].rmd.foot_vel_coarse - history[i].smd.foot_vel_coarse) * vel_coarse_error_factor;
		error += sq(history[i].rmd.error_p - history[i].smd.error_p - delta);
	}
	if (end_out == 0)
		end_out = end;
	return error / (end-start);
}

bool control_draw_ui(bool* pressing_key)
{
	set_plot_history_elements(main_plot, history.size(),
		[](s64 i){ return history[i].smd.display_time; },
		1.0 / odrive_frequency);

	s64 plot_start_history = plot_get_visual_selection_index(main_plot);

	float monitor_height = 150*dpi_scaling;
	float sidebar_width = 350*dpi_scaling;

	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImVec2(sidebar_width, monitor_height));
	ImGui::Begin("Monitoring", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	{
	}
	ImGui::End();






	ImGui::SetNextWindowPos(ImVec2(0, monitor_height));
	ImGui::SetNextWindowSize(ImVec2(sidebar_width, (float)window_size_y-monitor_height));
	ImGui::Begin("UI", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

	//static float width = 1;
	//ImGui::DragFloat("width", &width, 0.1f, 0, 0, "%.3f");
	ImGui::PushItemWidth(180);

	

	if (ImGui::Button("convert history") || history.size() <= 1)
	{
		std::vector<MonitorDataEx>& h = get_controller_history();
		history.clear();
		for (int i = 0; i < h.size(); i++)
		{
			SimRealData srd;
			srd.smd = convert_history_item(h[i], i ? &h[i-1] : nullptr);
			if (i != 0)
			{
				//srd.smd.theta_dot = (h[i].imu_x_angle - h[i-1].imu_x_angle) / h[i].delta_time;
				srd.smd.theta_dot = lerp(
						(h[i].imu_x_angle - h[i-1].imu_x_angle) / max(h[i].delta_time, 0.01f),
						history.back().smd.theta_dot,
						scd.theta_dot_smooth);
			}
			srd.rmd = srd.smd;
			history.push_back(srd);
		}
	}
	if (ImGui::Button("Simplify cd"))
	{
		cd.squat_upper_leg_cog_x = 0;
		cd.squat_lower_leg_cog_x = 0;
		//cd.squat_base_angle2     = 0;
	}

	{
		int e = simulate_world + simulate_policy*2;
		ImGui::TextDisabled("Simulate:");
		ImGui::RadioButton("None", &e, 0); ImGui::SameLine();
		ImGui::RadioButton("Policy", &e, 2); ImGui::SameLine();
		ImGui::RadioButton("World", &e, 1); ImGui::SameLine();
		ImGui::RadioButton("Both", &e, 3);
		simulate_world = e%2;
		simulate_policy = e/2;
		//ImGui::Checkbox("simulate_world", &simulate_world);
		//ImGui::Checkbox("simulate_policy", &simulate_policy);
	}

	bool do_interactive_changed = ImGui::Checkbox("do_interactive", &do_interactive);
	
	ImGui::SliderInt("warmup steps" , &simulation_warmup_steps, 0, 50);

	if (simulate_world || simulate_policy)
		run_simulation();

	if (do_interactive)
	{
		SimRealData srd;
		if (do_interactive_changed)
		{
			// start interactive simulation. Reset robot state.
			auto& md = history[plot_start_history].smd;
			srd.smd = SimMonitorData();
			srd.smd.common_hip_pos_target = md.common_hip_pos_target;
			srd.smd.common_hip_pos_sensor = md.common_hip_pos_sensor;
			srd.smd.common_leg_pos_target = md.common_leg_pos_target;
			srd.smd.common_leg_pos_target_odrv = md.common_leg_pos_target;
			srd.smd.common_leg_pos = md.common_leg_pos;
			srd.smd.imu_x_angle = calculate_squat_angle(srd.smd.common_hip_pos_sensor, srd.smd.common_leg_pos);
			srd.smd.imu_x_angle_interframe = srd.smd.imu_x_angle;
			srd.smd.balance_control_target_angle = srd.smd.imu_x_angle;
			srd.smd.balance_control_gain_p               = scd.balance_control_gain_p;
			srd.smd.balance_control_gain_i               = scd.balance_control_gain_i;
			srd.smd.balance_control_gain_d               = scd.balance_control_gain_d;
			srd.smd.balance_control_gain_vel_high        = scd.balance_control_gain_vel_high;
			srd.smd.balance_control_gain_vel_low         = scd.balance_control_gain_vel_low;
			srd.smd.balance_control_gain_backcalculation = scd.balance_control_gain_backcalculation;
			srd.smd.sim_target_x_angle_delta = -cd.squat_base_angle2+scd.sim_target_x_angle_delta;

			{
				float th_pos = srd.smd.error_p;

				float leg_pos = srd.smd.common_leg_pos;
				float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;
				float hip_angle = hip_angle_0 + scd.hip_sensor_angle_range*srd.smd.common_hip_pos_sensor;
				float foot_pos  = srd.smd.foot_pos;
				float coi_x;
				get_com_world_position(srd.smd.imu_x_angle, th_pos,
						hip_angle, leg_angle, foot_pos,
						hip_angle, leg_angle, foot_pos,
						0, 0,
						&coi_x, 0,
						0);
		
				srd.smd.coi_world_pos = coi_x;
			}
			
			// todo: update end of plot?
			simulate_world = false;
			simulate_policy = false;
		}
		else
			srd.smd = history.back().smd;

		srd.smd.display_time = history.back().smd.display_time + scd.target_delta_time_ms * odrive_frequency / 1000;
		
		do_interactive_keys(srd.smd, pressing_key);
		body_acc_update(srd.smd);
		sim_update(srd.smd);
		{
			SimMonitorData& history_md = history[max((int)history.size()-1-cd.coi_theta_past-1, 0)].smd;
			float history_theta = history_md.error_p + history_md.t_correction_angle - history_md.t_correction_angle_raw;
			sim_policy(srd.smd, history_theta);
		}

		srd.rmd = srd.smd;
		history.push_back(srd);
	}

	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Monitor Data", ImGuiTreeNodeFlags_DefaultOpen))
	{
		for (int real = 0; real < 2; real++)
		{
			PLOT_HISTORY("control_vel", md.balance_control_vel_user*0.2f*scd.control_vel_factor);
			PLOT_HISTORY("control_vel_smooth", md.balance_control_vel_quasi_smooth*0.2f);
			PLOT_HISTORY("action", md.balance_control_vel_output*0.2f);
			PLOT_HISTORY("x_angle", md.imu_x_angle);
			PLOT_HISTORY("error_p", md.error_p);
			PLOT_HISTORY("error_d", md.error_d);
			PLOT_HISTORY("theta_dot", md.theta_dot);
			PLOT_HISTORY("theta_acc", md.theta_acc);
			PLOT_HISTORY("target_angle", md.balance_control_target_angle);
			PLOT_HISTORY("target_angle_vel", md.balance_control_target_angle_vel);
			PLOT_HISTORY("common_hip_pos_target", md.common_hip_pos_target);
			PLOT_HISTORY("common_hip_pos_sensor", md.common_hip_pos_sensor);
			PLOT_HISTORY("common_leg_pos_target", md.common_leg_pos_target);
			//PLOT_HISTORY("common_leg_pos_target_odrv", md.common_leg_pos_target_odrv);
			PLOT_HISTORY("common_leg_pos", md.common_leg_pos);
			PLOT_HISTORY("common_leg_pos_dot .1x", md.common_leg_pos_dot*0.1f);
			PLOT_HISTORY("sensor_to_floor_pos_y .01x", -md.imu_sensor_to_floor_pos_y*0.01f-2.2f);
			PLOT_HISTORY("sensor_to_floor_vel_y .001x", -md.imu_sensor_to_floor_vel_y*0.001f);
			PLOT_HISTORY("l_vel_target .1x", md.l_vel_target*0.1f);
			PLOT_HISTORY("l_leg_height_pos", md.l_leg_height_pos);
			PLOT_HISTORY("l_leg_height_vel .1x", md.l_leg_height_vel*0.1f);
			PLOT_HISTORY("squat_length", md.squat_length*0.001f);
			PLOT_HISTORY("foot_pos", md.foot_pos);
			//PLOT_HISTORY("foot_acc", md.foot_acc);
			PLOT_HISTORY("t_correction_angle x5", md.t_correction_angle*5);
			PLOT_HISTORY("t_correction_angle_raw x5", md.t_correction_angle_raw*5);
			PLOT_HISTORY("sim_target_x_angle_delta x5", md.sim_target_x_angle_delta*5);
			PLOT_HISTORY("foot_vel_target", md.foot_vel_target*0.2f);
			PLOT_HISTORY("foot_vel_coarse", md.foot_vel_coarse*0.2f);
			PLOT_HISTORY("foot_current .05", md.foot_current*0.05f);
		
			PLOT_HISTORY("jump_phase", (float)md.jump_phase*0.2f);

			PLOT_HISTORY("leg_current .05", md.leg_current*0.05f);
			PLOT_HISTORY("leg_integrator .05", md.leg_integrator*0.05f);

			//PLOT_HISTORY("floor_vel", (md.foot_vel_coarse_interframe*tau + md.theta_dot + md.common_hip_pos_sensor_dot + md.common_leg_pos_dot*(tau / leg_gear_reduction))/tau*0.2f);
			//PLOT_HISTORY("delta_pos", md.balance_control_delta_pos*0.2f);
			PLOT_HISTORY("delta_vel", md.balance_control_delta_vel*0.2f);
			PLOT_HISTORY("vel_p", md.vel_p*0.2f);
			PLOT_HISTORY("vel_d", md.vel_d*0.2f);
			PLOT_HISTORY("vel_pos", md.vel_pos*0.2f);
			PLOT_HISTORY("balance_control_integrator", md.balance_control_integrator*0.2f);
			PLOT_HISTORY("vel_backcalculation", md.vel_backcalculation*0.2f);
			PLOT_HISTORY("position_control_error", md.balance_control_position_control_error*0.2f);
			PLOT_HISTORY("enable_position_control", md.balance_control_enable_position_control);
			PLOT_HISTORY("is_standing", md.is_standing);
			//PLOT_HISTORY("standing_timer", md.balance_control_standing_action_tolerance_timer);
			ImGui::NewLine();

			PLOT_HISTORY("coi_pos", md.coi_world_pos);
			PLOT_HISTORY("coi_vel", md.coi_world_vel/(tau*wheel_radius)*0.2f);
			PLOT_HISTORY("coi_acc", md.coi_world_acc);
			PLOT_HISTORY("theta acc", md.coi_theta_acc);
			ImGui::NewLine();

		}
		static bool show = false;
		plot_history(main_plot, "foot_vel_coarse_diff", [](s64 i){return history[i].smd.foot_vel_coarse-history[i].rmd.foot_vel_coarse;}, &show);
	}
	ImGui::DragFloat("coi vel smooth fac", &cd.coi_world_vel_smooth_factor, 0.01f, 0, 0, "%.5f");
	ImGui::DragFloat("coi acc smooth fac", &cd.coi_world_acc_smooth_factor, 0.01f, 0, 0, "%.5f");
	ImGui::DragInt("coi_theta_past", &cd.coi_theta_past);

	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Control Data", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::Checkbox("use_control_gains", &use_control_gains);
		ImGui::DragFloat("error_d_smooth", &scd.balance_control_error_d_smooth, 0.01f);
		ImGui::DragFloat("target_angle_vel_gain"  , &scd.balance_control_target_angle_vel_gain, 0.01f);
		ImGui::DragFloat("target_angle_vel_smooth", &scd.balance_control_target_angle_vel_smooth, 0.01f);
		ImGui::DragFloat("gain_p", &scd.balance_control_gain_p, 0.01f);
		ImGui::DragFloat("gain_i", &scd.balance_control_gain_i, 0.1f);
		ImGui::DragFloat("gain_d", &scd.balance_control_gain_d, 0.01f);
		ImGui::DragFloat("gain_vel_high", &scd.balance_control_gain_vel_high, 0.01f);
		ImGui::DragFloat("gain_vel_low", &scd.balance_control_gain_vel_low, 0.01f);
		ImGui::Checkbox("use_coi_vel", &scd.balance_control_use_coi_vel);
		ImGui::DragFloat("gain_backcalculation", &scd.balance_control_gain_backcalculation, 0.1f);
		ImGui::DragFloat("max_theta_integrator", &scd.balance_control_max_theta_integrator, 0.1f);
		ImGui::DragFloat("max_balance_integrator", &scd.max_balance_control_integrator, 0.1f);
		bool b1 = ImGui::DragFloat("theta_dot_smooth", &scd.theta_dot_smooth, 0.01f);
		if (b1)
		{
			const float delta_time = scd.target_delta_time_ms * 0.001f;

			history[0].smd.theta_dot = 0;
			for (int i = 1; i < history.size(); i++)
			{
				history[i].smd.theta_dot = lerp(
						(history[i].smd.imu_x_angle - history[i-1].smd.imu_x_angle) / delta_time,
						history[i-1].smd.theta_dot,
						scd.theta_dot_smooth);
			}
			history[history.size()-1].rmd.theta_dot = 0;
			for (int i = (int)history.size()-2; i >= 0; i--)
			{
				history[i].rmd.theta_dot = lerp(
						(history[i+1].smd.imu_x_angle - history[i].smd.imu_x_angle) / delta_time,
						history[i+1].rmd.theta_dot,
						scd.theta_dot_smooth);
			}
			for (int i = 1; i < history.size(); i++)
			{
				history[i].smd.theta_dot = (history[i].smd.theta_dot + history[i].rmd.theta_dot) * 0.5f;
				history[i].rmd.theta_dot = history[i].smd.theta_dot;
			}
		}
		ImGui::DragFloat("theta_acc_smooth", &scd.theta_acc_smooth, 0.01f);

		ImGui::DragFloat("vel_quasi_smooth_factor", &scd.balance_control_vel_quasi_smooth_factor, 0.01f);
		ImGui::DragFloat("vel_quasi_smooth_max_acc", &scd.balance_control_vel_quasi_smooth_max_acc, 0.01f);
		ImGui::DragFloat("standing_action_tolerance", &scd.balance_control_standing_action_tolerance, 0.01f);
		ImGui::DragFloat("standing_action_tolerance_time", &scd.balance_control_standing_action_tolerance_time, 0.01f);
		ImGui::DragFloat("vel_acceleration_limit", &scd.balance_control_vel_acceleration_limit, 0.01f);
		ImGui::DragFloat("control_vel_factor", &scd.control_vel_factor, 0.01f);
		ImGui::DragFloat("fake_x_angle_delta_vel", &scd.fake_x_angle_delta_vel, 0.01f);
		ImGui::DragFloat("sim_target_x_angle_delta", &scd.sim_target_x_angle_delta, 0.001f);
		ImGui::DragFloat("foot_control_vel_limit", &scd.foot_control_vel_limit, 0.01f);
		ImGui::DragFloat("foot_control_accel_limit", &scd.foot_control_accel_limit, 0.01f);
		ImGui::DragFloat("t_correction_simple_factor", &cd.t_correction_simple_factor, 0.01f);
		ImGui::DragFloat("t_correction_acc_p", &cd.t_correction_acc_p, 0.01f);
		ImGui::DragFloat("imu_sensor_to_floor_vel_smooth_factor", &scd.imu_sensor_to_floor_vel_smooth_factor, 0.01f);
		
		ImGui::DragInt  ("position_control_after_frame", &scd.balance_control_position_control_after_frame);
		ImGui::DragFloat("position_control_target", &scd.balance_control_position_control_target, 0.01f);
		ImGui::DragFloat("position_control_max_vel", &scd.balance_control_position_control_max_vel, 0.01f);
		ImGui::DragFloat("position_control_gain_p_high", &scd.balance_control_position_control_gain_p_high, 0.01f);
		ImGui::DragFloat("position_control_gain_d_high", &scd.balance_control_position_control_gain_d_high, 0.01f);
		ImGui::DragFloat("position_control_gain_p_low", &scd.balance_control_position_control_gain_p_low, 0.01f);
		ImGui::DragFloat("position_control_gain_d_low", &scd.balance_control_position_control_gain_d_low, 0.01f);

		if (ImGui::Button("reset control data"))
			scd = SimControlData();
	}
	
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("World Model Parameters", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::DragFloat("initial_sim_theta", &initial_sim_theta, 0.0001f, 0, 0, "%.4f");
		ImGui::DragFloat("initial_sim_theta_dot", &initial_sim_theta_dot, 0.0001f, 0, 0, "%.4f");
		ImGui::DragFloat("initial_foot_integrator", &initial_foot_integrator, 0.01f, 0, 0, "%.4f");
		ImGui::DragFloat("initial_balance_control_integrator", &initial_balance_control_integrator, 0.01f, 0, 0, "%.4f");

		ImGui::DragFloat("foot_vel_gain", &scd.foot_control_vel_gain, 0.01f);
		ImGui::DragFloat("foot_vel_integrator_gain", &scd.foot_control_vel_integrator_gain, 0.01f);
		ImGui::DragFloat("foot_motor_kv", &scd.foot_control_motor_kv, 1);
		ImGui::DragFloat("leg_motor_kv", &scd.leg_control_motor_kv, 1);
		ImGui::DragFloat("current_lim", &scd.current_lim, 0.01f);
		ImGui::NewLine();

		ImGui::DragInt("target_delta_time_ms", &scd.target_delta_time_ms, 1, 1, 200);
		ImGui::DragInt("num_sim_iterations", &scd.num_sim_iterations, 1, 1, 200);
		ImGui::DragFloat("vel_sensor_delay", &scd.vel_sensor_delay, 0.001f, 0, 1);
		ImGui::DragFloat("imu_sensor_delay", &scd.imu_sensor_delay, 0.001f, 0, 1);
		ImGui::DragFloat("hip_motor_p_gain", &scd.hip_motor_p_gain, 0.01f);
		ImGui::DragFloat("hip_motor_d_gain", &scd.hip_motor_d_gain, 0.01f);
		ImGui::DragFloat("leg_motor_p_gain", &scd.leg_motor_p_gain, 0.01f);
		ImGui::DragFloat("leg_motor_d_gain", &scd.leg_motor_d_gain, 0.01f);
		ImGui::NewLine();

		ImGui::DragFloat("squat_whole_body_inertia", &cd.squat_whole_body_inertia, 0.001f);
		ImGui::DragFloat("squat_wheel_inertia"    , &cd.squat_wheel_inertia, 0.00001f, 0, 0, "%.5f");
		ImGui::DragFloat("squat_body_mass"        , &cd.squat_body_mass      , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("squat_upper_leg_mass"   , &cd.squat_upper_leg_mass , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("squat_lower_leg_mass"   , &cd.squat_lower_leg_mass , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("squat_wheel_mass"       , &cd.squat_wheel_mass     , 0.001f, 0, 0, "%.3f");
		ImGui::NewLine();

		ImGui::DragFloat("sim_squat_body_cog_x"         , &scd.sim_squat_body_cog_x         , 1, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_body_cog_y"         , &scd.sim_squat_body_cog_y         , 1, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_body_mass"          , &scd.sim_squat_body_mass          , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_body_inertia"       , &scd.sim_squat_body_inertia       , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_upper_leg_cog_y"    , &scd.sim_squat_upper_leg_cog_y    , 1, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_upper_leg_mass"     , &scd.sim_squat_upper_leg_mass     , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_upper_leg_inertia"  , &scd.sim_squat_upper_leg_inertia  , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_lower_leg_cog_y"    , &scd.sim_squat_lower_leg_cog_y    , 1, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_lower_leg_mass"     , &scd.sim_squat_lower_leg_mass     , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_lower_leg_inertia"  , &scd.sim_squat_lower_leg_inertia  , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_wheel_mass"         , &scd.sim_squat_wheel_mass         , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_wheel_inertia"      , &scd.sim_squat_wheel_inertia      , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_rubber_inertia"     , &scd.sim_squat_rubber_inertia     , 0.001f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_rubber_factor"      , &scd.sim_squat_rubber_factor      , 0.1f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_body_inertia_factor", &scd.sim_squat_body_inertia_factor, 0.1f, 0, 0, "%.3f");
		ImGui::DragFloat("sim_squat_body_mass_factor"   , &scd.sim_squat_body_mass_factor   , 0.1f, 0, 0, "%.3f");
		ImGui::NewLine();

		if (ImGui::Button("reset model data"))
		{
			SimControlData c = SimControlData();
			scd.foot_control_vel_gain = c.foot_control_vel_gain;
			scd.foot_control_vel_integrator_gain = c.foot_control_vel_integrator_gain;
			scd.num_sim_iterations = c.num_sim_iterations;
			scd.vel_sensor_delay = c.vel_sensor_delay;
			scd.imu_sensor_delay = c.imu_sensor_delay;
			scd.sim_squat_body_cog_x          = c.sim_squat_body_cog_x         ;
			scd.sim_squat_body_cog_y          = c.sim_squat_body_cog_y         ;
			scd.sim_squat_body_mass           = c.sim_squat_body_mass          ;
			scd.sim_squat_body_inertia        = c.sim_squat_body_inertia       ;
			scd.sim_squat_upper_leg_cog_y     = c.sim_squat_upper_leg_cog_y    ;
			scd.sim_squat_upper_leg_mass      = c.sim_squat_upper_leg_mass     ;
			scd.sim_squat_upper_leg_inertia   = c.sim_squat_upper_leg_inertia  ;
			scd.sim_squat_lower_leg_cog_y     = c.sim_squat_lower_leg_cog_y    ;
			scd.sim_squat_lower_leg_mass      = c.sim_squat_lower_leg_mass     ;
			scd.sim_squat_lower_leg_inertia   = c.sim_squat_lower_leg_inertia  ;
			scd.sim_squat_wheel_mass          = c.sim_squat_wheel_mass         ;
			scd.sim_squat_wheel_inertia       = c.sim_squat_wheel_inertia      ;
			scd.sim_squat_rubber_inertia      = c.sim_squat_rubber_inertia     ;
			scd.sim_squat_rubber_factor       = c.sim_squat_rubber_factor      ;
			scd.sim_squat_body_inertia_factor = c.sim_squat_body_inertia_factor;
			scd.sim_squat_body_mass_factor    = c.sim_squat_body_mass_factor   ;
		}
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Simulate Landing", 0))
	{
		ImGui::SliderInt("steps" , &simulate_landing_steps, 0, 100);
		ImGui::DragFloat("common_leg_pos ", &landing_common_leg_pos, 0.01f);
		ImGui::DragFloat("common_leg_pos_dot", &landing_common_leg_pos_dot, 0.01f);
		ImGui::DragFloat("imu_x_angle", &landing_imu_x_angle, 0.01f);
		ImGui::DragFloat("imu_x_angle_dot", &landing_imu_x_angle_dot, 0.01f);
		ImGui::DragFloat("foot_vel", &landing_foot_vel, 0.01f);
		ImGui::DragFloat("balance_control_integrator ", &landing_balance_control_integrator, 0.01f);
		ImGui::NewLine();

		ImGui::DragFloat("l_pos_zero_delta", &scd.l_pos_zero_delta            , 0.01f);
		ImGui::DragFloat("l_pos_zero_max", &scd.l_pos_zero_max              , 0.01f);
#ifdef NEW_LANDING
		ImGui::DragFloat("l_base_angle_delta", &scd.l_base_angle_delta        , 0.01f);
		ImGui::DragFloat("l_max_time", &scd.l_max_time        , 0.01f);
#endif
		ImGui::DragFloat("l_vel_gain", &scd.l_vel_gain                  , 0.01f);
		ImGui::DragFloat("l_pos_gain", &scd.l_pos_gain                  , 0.01f);
		ImGui::DragFloat("l_vel_integrator_gain", &scd.l_vel_integrator_gain       , 0.01f);
		ImGui::DragFloat("l_vel_limit", &scd.l_vel_limit , 0.01f);
		ImGui::DragFloat("f_current_lim", &scd.f_current_lim, 0.01f);
		ImGui::DragFloat("pos_err_sync_gain", &scd.pos_err_sync_gain , 0.01f);
		ImGui::DragFloat("landing_fixed_foot_vel_time", &scd.landing_fixed_foot_vel_time , 0.01f);
		ImGui::DragFloat("landing_fixed_foot_vel", &scd.landing_fixed_foot_vel , 0.01f);
		ImGui::DragFloat("gain_vel_landing_warmup_time", &scd.balance_control_gain_vel_landing_warmup_time, 0.01f);
	}
	ImGui::NewLine();

	if (ImGui::CollapsingHeader("Optimize World Model"))
	{
		static int optimizer_max_steps = 30;
		ImGui::DragInt("max_steps", &optimizer_max_steps);

		static int optimizer_iterations = 100;
		ImGui::DragInt("optimizer_iterations", &optimizer_iterations, 1, 1, 1000);

		s64 start_, end_;
		plot_get_display_range(main_plot, &start_, &end_);
		int start = (int)start_, end = (int)end_;

		start += simulation_warmup_steps;
		end = 0;
		float error = sim_model_error(start, end, optimizer_max_steps);
		ImGui::TextDisabled("Steps: %d", end-start);
		ImGui::TextDisabled("Error: %f", error);

		static float epsilon = 0.000001f;
		ImGui::DragFloat("epsilon", &epsilon, 0.0001f, 0, 0, "%.6f");
		static float lr = 0.01f;
		ImGui::DragFloat("lr", &lr, 0.0001f, 0, 0, "%.6f");
		static float alpha = 0.9f;
		ImGui::DragFloat("alpha", &alpha, 0.0001f, 0, 0, "%.6f");

		ImGui::Checkbox("optimize only initial", &optimize_only_initial);
		ImGui::Checkbox("optimize with delta", &optimize_with_delta);
		ImGui::DragFloat("vel_coarse_error_factor", &vel_coarse_error_factor, 0.0001f, 0, 0, "%.6f");

		static bool optimize = false;
		ImGui::Checkbox("Optimize", &optimize);
		if (optimize)
		{
			for (int i = 0; i < optimizer_iterations; i++)
			{
				run_simulation();
				error = sim_model_error(start, end, optimizer_max_steps);
				vector<float> gradients;
				each_world_model_var([&](float& var)
					{
						float old_var = var;
						var += epsilon;
						run_simulation();
						float new_error = sim_model_error(start, end, optimizer_max_steps);
						var = old_var;
						gradients.push_back((new_error-error) / epsilon);
					});

				static vector<float> momentum(gradients.size());
				if (optimizer_iterations == 1)
					memset(momentum.data(), 0, momentum.size()*sizeof(float));

				int index = 0;
				each_world_model_var([&](float& var)
					{
						float delta = momentum[index]*alpha - gradients[index]*lr;
						momentum[index] = delta;
						var += delta;
						index++;
					});
			}
		}
	}
	ImGui::NewLine();
	
	ImGui::PopItemWidth();
	ImGui::End();

	{
		ImGui::SetNextWindowPos(ImVec2(sidebar_width, 0));
		ImGui::SetNextWindowSize(ImVec2((float)window_size_x-sidebar_width+5, (float)window_size_y));
		ImGui::Begin("Main", nullptr, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoBackground);

		if (ImGui::TreeNodeEx("Robot visualization", ImGuiTreeNodeFlags_DefaultOpen))
		{
			auto& md = history[plot_start_history].smd;
			static bool show_fixed = true;
			ImGui::Checkbox("show fixed", &show_fixed);
			ImGui::SameLine();
			static float foot_pos_visual_reference = 0;
			if (ImGui::Button("reset robot position"))
				foot_pos_visual_reference = -md.rubber_pos + (show_fixed ? 0 : md.balance_control_pos_user);

			float hip_angle = hip_angle_0+scd.hip_sensor_angle_range*md.common_hip_pos_sensor;
			float leg_pos = md.common_leg_pos;
			float leg_angle = (1-leg_pos) * (tau / leg_gear_reduction) + leg_angle_1;

			ImVec2 draw_robot_ui_pos = ImGui::GetCursorScreenPos();
			ImGui::Dummy(ImVec2(0, 100*dpi_scaling));

			draw_robot(draw_robot_ui_pos.x+400*dpi_scaling, draw_robot_ui_pos.y+100*dpi_scaling,
					md.imu_x_angle_interframe,
					hip_angle, leg_angle,
					hip_angle, leg_angle,
					foot_pos_visual_reference, show_fixed ? 0 : md.balance_control_pos_user,
					md.imu_sensor_to_floor_pos_x, md.imu_sensor_to_floor_pos_y, md);
			ImGui::TreePop();
		}
		ImGui::NewLine();
		ImGui::End();
		
		draw_ui_main_2();
	}
	return true;
}

void control_init()
{
	history.push_back({});
}
