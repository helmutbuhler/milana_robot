// The leaning, or more specifically the body movement to the left and right, of the robot can be
// directly controlled with the xbox controller (and also with the ui in control_ui).
// To implement this requires basically solving two problems:
// - Calculating the target angles of the leg and hip motors to reach a certain leaning angle.
// - Making sure the robot doesn't fall over due to the leaning.
//
// The first part is implemented in geometry.cpp, as part of the IK.
// The second part is implemented here.
// To do this, we limit the leaning to a fixed range and calculate how fast the robot is able
// to rotate (difference of foot motors speed) with a given leaning angle. This boils down to solving
// a quadratic equation, which I did on paper and converted the solution equation into code.
// (Solving this is not as easy as one might think at first)
//
// Another thing that is done here is to augment the leaning angle when the user rotates the robot
// while it is moving. There is a simple formula to calculate this and it basically results in the
// robot leaning into a curve (similar to a motorbike in a curve) and allows for better control
// when moving fast.

#include "leaning.h"
#include "helper.h"
#include "geometry.h"
#include <vector>
#include <algorithm>
#include <cstring>
#ifndef COMPILING_CONTROL_UI
#include "../robot/main.h"
#endif

using std::vector;

extern ControlData cd;

bool leaning_init()
{
	return true;
}


void calculate_rotation_vel_range(float h, float v, float imu_side_angle, float& r_min, float& r_max)
{
	float g = earth_acc*1000;
	float d = cd.wheel_side_distance;
	float alpha = tan(imu_side_angle);

	float x2 = -(8*h*h*alpha)/(d*d*d*g);
	float x1 = (4*h*v)/(d*d*g);
	float x0 = -(2*h*alpha)/(d);
	x1 /= x2;
	
	float r0_radicand = sq(x1/2)-(x0-cd.leaning_angle_rot_tolerance)/x2;
	float r1_radicand = sq(x1/2)-(x0+cd.leaning_angle_rot_tolerance)/x2;
	float r00 = -x1/2 + sqrt(r0_radicand);
	float r01 = -x1/2 - sqrt(r0_radicand);
	float r10 = -x1/2 + sqrt(r1_radicand);
	float r11 = -x1/2 - sqrt(r1_radicand);

	if (r0_radicand >= 0 && r1_radicand >= 0)
	{
		float r0_dist = std::min(abs(r00), abs(r10));
		float r1_dist = std::min(abs(r01), abs(r11));
		float y0, y1;
		if (r0_dist < r1_dist)
		{
			y0 = r00;
			y1 = r10;
		}
		else
		{
			y0 = r01;
			y1 = r11;
		}
		r_min = std::min(y0, y1);
		r_max = std::max(y0, y1);
	}
	else if (r0_radicand >= 0)
	{
		r_max = r00;
		r_min = r01;
	}
	else
	{
		r_max = r10;
		r_min = r11;
	}
	r_min /= tau*wheel_radius;
	r_max /= tau*wheel_radius;
}

void calculate_rotation_vel_range_safe(const MonitorData& md, float h, float& r_min, float& r_max)
{
	r_min = -1000;
	r_max =  1000;
	for (int i = -1; i <= 1; i++)
	for (int j = 0; j < 2; j++)
	{
		float v = (md.balance_control_vel_output+i*cd.leaning_vel_tolerance)*tau*wheel_radius;
		
		// Theoretically, we just need imu_side_angle for the following calculation.
		// But, during fast rotations imu_side_angle and error_p practically swap places
		// without giving us time to react.
		// To be safe, we add both angles.
		// During movement with constant velocity, error_p will be practically 0.
		float side_angle = md.imu_side_angle + md.error_p*(j ? 1 : -1);
		
		float l_min, l_max;
		calculate_rotation_vel_range(h, v, side_angle, l_min, l_max);
		r_min = std::max(r_min, l_min);
		r_max = std::min(r_max, l_max);
	}

	// Make sure 0 is included in the range.
	if (r_min > 0) r_min = 0;
	if (r_max < 0) r_max = 0;
}


#ifndef COMPILING_CONTROL_UI
bool leaning_update()
{
	md.leaning_target_angle = 0;

	if (cd.servo_manual_enable) return true;
	if (md.jump_phase != JP_None && md.jump_phase != JP_PostLanding) return true;

	float com_length;
	calculate_squat_angle(md.common_hip_pos_target, md.common_leg_pos_target, &com_length);

	// Update hip position
	if (cd.servo_enable)
	{
		md.common_servo_vel = cd.common_servo_target_vel_user;
		md.common_servo_vel = clamp(md.common_servo_vel, -cd.servo_vel_max, cd.servo_vel_max);
		md.common_hip_pos_target += md.common_servo_vel*md.delta_time;
		md.common_hip_pos_target = clamp(md.common_hip_pos_target, 0, 1);
	}

	// Update leg position
	if (cd.enable_stand_control)
	{
		if (md.startup_sequence == SS_FinalLegHipMovement)
		{
			md.common_leg_pos_target = move_towards(md.common_leg_pos_target,
					cd.startup_leg_final_pos, cd.common_leg_vel_user_max*md.delta_time);
			md.common_leg_vel_target = cd.common_leg_vel_user_max;
		}
		else
		{
			md.common_leg_pos_target += cd.common_leg_vel_target*md.delta_time;
			md.common_leg_vel_target = cd.common_leg_vel_target;
		}
		md.common_leg_pos_target = clamp(md.common_leg_pos_target, cd.common_leg_pos_min, cd.common_leg_pos_max);
	}

	// Calculate desired leaning angle
	{
		// In case the robot moves very fast and rotates at the same time,
		// we compensate the centrifugal force by leaning.
		// leaning formula: https://www.physicsforums.com/threads/motorcycle-lean-angle-speed-and-turn-radius.13060/
		md.leaning_target_angle = atan(2*cd.balance_control_rotation_vel_user_target*md.balance_control_vel_quasi*sq(tau*wheel_radius) / (cd.wheel_side_distance*earth_acc*1000));

		// Add angle, if user explicitly asks for it
		md.leaning_target_angle += cd.leaning_target_angle_user + md.leaning_target_angle_joystick;

		// If the absolute angle gets too big the rubber wheels lose contact with the floor, so we clamp it.
		float leaning_target_angle_clamped = clamp(md.leaning_target_angle, -cd.leaning_max_angle, cd.leaning_max_angle);

		// We smooth the target_angle trajectory. Otherwise a quick movement might gain so much momentum
		// in side direction that the robot falls to the side.
		float new_target_angle_smooth = update_smooth(
				md.leaning_target_angle_smooth,
				leaning_target_angle_clamped,
				cd.leaning_target_angle_smooth_factor,
				cd.target_delta_time_ms);

		// We also limit the maximum velocity.
		float max_vel = clamp_and_map(com_length+wheel_radius, 170, 220,
			cd.leaning_target_angle_max_vel_low, cd.leaning_target_angle_max_vel_high);
		md.leaning_target_angle_smooth = clamp(
				new_target_angle_smooth,
				md.leaning_target_angle_smooth-max_vel*md.delta_time,
				md.leaning_target_angle_smooth+max_vel*md.delta_time);
	}

	// Calculate safe range of rotation velocity
	calculate_rotation_vel_range_safe(md, com_length+wheel_radius,
			md.leaning_min_rotation_vel, md.leaning_max_rotation_vel);
	
	return true;
}
#endif

void leaning_close()
{
}

