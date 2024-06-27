// Here we calculate the expected acceleration of the robot, based on how fast the joint motors
// are moving. We need this to properly calculate the gravity acceleration vector in sensor_fusion.cpp

#include "body_acc.h"
#include "../common/helper.h"
#include "main.h"
#include "../common/geometry.h"
#include <stdio.h>
#include <math.h>


bool body_acc_init()
{
	return true;
}

bool body_acc_update()
{
	// We use the sensor values from the last frame here, because
	// imu sensor calculation happens very early in the main loop and almost everyone needs
	// current imu values. But that's ok because this data
	// is slowly changing and quite noisy anyway.

	float hip_angle_l = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
	float hip_angle_r = hip_angle_0+cd.hip_sensor_angle_range*md.hip_sensor_pos[1];
	float foot_pos_l  = -md.foot_pos[1];
	float foot_pos_r  =  md.foot_pos[0];
	float leg_pos_l   =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
	float leg_pos_r   = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
	float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
	float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;

	// First we calculate the position of the imu sensor, relative to a fixed point on the floor.
	float imu_x, imu_y;
	get_imu_sensor_position(md.imu_x_angle,
			hip_angle_l, leg_angle_l, foot_pos_l,
			hip_angle_r, leg_angle_r, foot_pos_r,
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
			(md.imu_sensor_to_floor_pos_x-old_imu_sensor_to_floor_pos_x) / md.delta_time,
			cd.imu_sensor_to_floor_vel_smooth_factor, cd.target_delta_time_ms);
	md.imu_sensor_to_floor_vel_y = update_smooth(
			old_imu_sensor_to_floor_vel_y,
			(md.imu_sensor_to_floor_pos_y-old_imu_sensor_to_floor_pos_y) / md.delta_time,
			cd.imu_sensor_to_floor_vel_smooth_factor, cd.target_delta_time_ms);

	float old_imu_sensor_to_floor_acc_x = md.imu_sensor_to_floor_acc_x;
	float old_imu_sensor_to_floor_acc_y = md.imu_sensor_to_floor_acc_y;
	md.imu_sensor_to_floor_acc_x = update_smooth(
			old_imu_sensor_to_floor_acc_x,
			(md.imu_sensor_to_floor_vel_x-old_imu_sensor_to_floor_vel_x) / md.delta_time / 1000 / earth_acc,
			cd.imu_sensor_to_floor_acc_smooth_factor, cd.target_delta_time_ms);
	md.imu_sensor_to_floor_acc_y = update_smooth(
			old_imu_sensor_to_floor_acc_y,
			(md.imu_sensor_to_floor_vel_y-old_imu_sensor_to_floor_vel_y) / md.delta_time / 1000 / earth_acc,
			cd.imu_sensor_to_floor_acc_smooth_factor, cd.target_delta_time_ms);
			
	return true;
}

void body_acc_close()
{
}

