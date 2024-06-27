// This code is responsible for taking the raw sensor values of the accelerometer and gyroscope
// (both in the IMU) and calculating an estimate for the vector of gravity (or in other words
// the rotation of the robot).
// Especially the jumping and the sidebalancing stuff require very precise and low latency values, so
// this code has gotten quite involved to make the calculation as precise as possible.
// - The gyro is compensated for static bias (measured on every startup) and bias correlated with measured temperature.
// - The measured acceleration is compensated for acceleration the robot causes.
// - The rotation of the imu sensor inside of the robot is take into account.
// - Both gyro and acceleration data is fused here (Basically Complementary Filter on 3D gravity vector)
// - During jumping, only gyro is used to keep track of rotation (accelerometer doesn't work in free fall).
// - The final gravity vector is converted into 2 angles. These two angles will then be used by balance control
//   and other pid controllers:
//   - x_angle: Angle in forward/backward direction
//   - side_angle: Angle in left/right direction


#include "../../common/helper.h"
#include "../main.h"
#include <math.h>
#include <stdio.h>

void do_sensor_fusion_gyro(MonitorData& md, bool constant_dt)
{
    //float gravity_length = sqrt(sq(md.imu_a_x) + 
    //                            sq(md.imu_a_y) + 
    //                            sq(md.imu_a_z));

	float delta_time = md.delta_time;
	if (constant_dt)
		delta_time = cd.target_delta_time_ms * 0.001f;

	switch (cd.imu_fifo_frequency)
	{
	case 0: delta_time *= 50.0f / 10; break;
	case 1: delta_time *= 50.0f / 25; break;
	case 2: delta_time *= 50.0f / 50; break;
	case 3: delta_time *= 50.0f / 100; break;
	case 4: delta_time *= 50.0f / 200; break;
	case 5: delta_time *= 50.0f / 400; break;
	case 6: delta_time *= 50.0f / 800; break;
	case 7: delta_time *= 50.0f / 1600; break;
	case 8: delta_time *= 50.0f / 3300; break;
	case 9: delta_time *= 50.0f / 6600; break;
	case 10:delta_time *= 50.0f / 13300; break;
	}

	// We apply two biases to the measured gyro values:
	// imu_gyro_bias_: Measured average values during bias estimation.
	// imu_temp_gyro_: Linear bias based on measured temperature.
	float delta_x = ((md.imu_gyro_x - md.imu_temp_gyro_x + md.imu_gyro_bias_x) * cd.imu_gyro_scale) * delta_time;
    float delta_y = ((md.imu_gyro_y - md.imu_temp_gyro_y + md.imu_gyro_bias_y) * cd.imu_gyro_scale) * delta_time;
    float delta_z = ((md.imu_gyro_z - md.imu_temp_gyro_z + md.imu_gyro_bias_z) * cd.imu_gyro_scale) * delta_time;

	Quaternion gyro_rotation;
	gyro_rotation.w = 1 - (sq(delta_x) + sq(delta_y) + sq(delta_z)) * 0.125f;
	gyro_rotation.x = -delta_x * 0.5f;
	gyro_rotation.y = -delta_y * 0.5f;
	gyro_rotation.z = -delta_z * 0.5f;

	// The above is a linear approximation of this
	// and is not only faster, but also seems to be working better.
	//gyro_rotation.w = cos((delta_x + delta_y + delta_z) * 0.5);
	//gyro_rotation.x = sin(-delta_x * 0.5);
	//gyro_rotation.y = sin(-delta_y * 0.5);
	//gyro_rotation.z = sin(-delta_z * 0.5);

	quaternion_rotate_vector(
            md.imu_gravity_x,
            md.imu_gravity_y,
            md.imu_gravity_z,
            gyro_rotation);
}

void do_sensor_fusion_acc()
{
	// The acceleration vector we measure is the sum of two vectors:
	// - gravity
	// - robot acceleration
	// We are only interested in gravity, so we need to estimate the robot acceleration
	// at the point of the imu sensor inside the robot and subtract it (see body_acc.cpp).
	md.imu_gravity_acc_x_comp = md.imu_gravity_acc_x;
	md.imu_gravity_acc_y_comp = md.imu_gravity_acc_y;
	md.imu_gravity_acc_z_comp = md.imu_gravity_acc_z;
	// (not sure about the minus and plus signs here. Figured them out by trial and error)
	md.imu_gravity_acc_y_comp += sin(md.imu_x_angle) * md.imu_sensor_to_floor_acc_x;
	md.imu_gravity_acc_z_comp -= cos(md.imu_x_angle) * md.imu_sensor_to_floor_acc_x;
	md.imu_gravity_acc_y_comp -= cos(md.imu_x_angle) * md.imu_sensor_to_floor_acc_y;
	md.imu_gravity_acc_z_comp -= sin(md.imu_x_angle) * md.imu_sensor_to_floor_acc_y;

   	if (md.jump_phase == JP_Jumping || md.jump_phase == JP_LegUp || md.jump_phase == JP_Falling ||
		cd.imu_force_no_acc)
	{
		// During a jump, the accelerometer returns mostly garbage data because of the acceleration
		// and because you cannot measure gravity during free fall.
	}
    else
    {
		if (cd.imu_compensate_body_acc && md.is_standing)
		{
			// Use gravity vector, compensated for robot acceleration.
			md.imu_gravity_x = md.imu_gravity_x * cd.imu_gyro_weight + md.imu_gravity_acc_x_comp;
			md.imu_gravity_y = md.imu_gravity_y * cd.imu_gyro_weight + md.imu_gravity_acc_y_comp;
			md.imu_gravity_z = md.imu_gravity_z * cd.imu_gyro_weight + md.imu_gravity_acc_z_comp;
		}
		else
		{
			md.imu_gravity_x = md.imu_gravity_x * cd.imu_gyro_weight + md.imu_gravity_acc_x;
			md.imu_gravity_y = md.imu_gravity_y * cd.imu_gyro_weight + md.imu_gravity_acc_y;
			md.imu_gravity_z = md.imu_gravity_z * cd.imu_gyro_weight + md.imu_gravity_acc_z;
		}
    }

	// Normalize gravity vector
    float gravity_length = sqrt(sq(md.imu_gravity_x) + 
                                sq(md.imu_gravity_y) + 
                                sq(md.imu_gravity_z));
    if (gravity_length < 0.001f)
    {
		// This should not happen in practice
        printf("reset imu!\n");
	    md.imu_gravity_x = md.imu_gravity_acc_x;
	    md.imu_gravity_y = md.imu_gravity_acc_y;
	    md.imu_gravity_z = md.imu_gravity_acc_z;
    }
    else
    {
        float one_by_length = 1.0f / gravity_length;
	    md.imu_gravity_x *= one_by_length;
	    md.imu_gravity_y *= one_by_length;
	    md.imu_gravity_z *= one_by_length;
    }

	float gravity_x_t, gravity_y_t, gravity_z_t;
	{
		// Because the imu sensor might be slightly rotated fixed inside the robot
		// we correct this by applying 3 euler rotations.
		float x0 = md.imu_gravity_x;
		float y0 = md.imu_gravity_y;
		float z0 = md.imu_gravity_z;
		float s, c;

		s = sin(cd.imu_bias_z);
		c = cos(cd.imu_bias_z);
		float x1 = x0 * c - y0 * s;
		float y1 = x0 * s + y0 * c;
		float z1 = z0;

		s = sin(cd.imu_bias_y);
		c = cos(cd.imu_bias_y);
		float x2 = x1 * c + z1 * s;
		float y2 = y1;
		float z2 =-x1 * s + z1 * c;

		s = sin(cd.imu_bias_x);
		c = cos(cd.imu_bias_x);
		float x3 = x2;
		float y3 = y2 * c - z2 * s;
		float z3 = y2 * s + z2 * c;

		gravity_x_t = x3;
		gravity_y_t = y3;
		gravity_z_t = z3;
	}
	
	// Calculate x_angle and side_angle
    md.imu_x_angle = (float)-atan2(gravity_z_t, -gravity_y_t);

   	float s = sin(md.imu_x_angle);
	float c = cos(md.imu_x_angle);
    float gravity_yz = -gravity_z_t * s - gravity_y_t * c;
	md.imu_side_angle = -(float)atan2(-gravity_x_t, gravity_yz);

	// Bias estimation
	// I tried implementing it in such a way that the bias is continuously updated, but I didn't get
	// it to work reliably. The problem is that there is always one axis of rotation where the gravity vector
	// doesn't provide any information about absolute rotation, and I wasn't able to figure out a way
	// around that.
	// Instead, we just keep all motors still for a brief moment during startup and measure the gyro drift
	// in that time. It's not elegant, but very reliable.
	// Note: We cannot just measure it once and then hardcode the bias. It seems to change on a daily/random
	// basis.
    static int last_imu_bias_estimation_trigger;
	if (cd.imu_bias_estimation_trigger-last_imu_bias_estimation_trigger == 1)
	{
        md.imu_do_bias_estimation = true;
	}
	last_imu_bias_estimation_trigger = cd.imu_bias_estimation_trigger;

    if (md.startup_sequence == SS_ImuCalibration1)
    {
        md.imu_do_bias_estimation = true;
        md.startup_sequence = SS_ImuCalibration2;
    }

	if (md.imu_do_bias_estimation)
    {
	    md.imu_bias_estimation_gyro_sum_x += md.imu_gyro_x-md.imu_temp_gyro_x;
	    md.imu_bias_estimation_gyro_sum_y += md.imu_gyro_y-md.imu_temp_gyro_y;
	    md.imu_bias_estimation_gyro_sum_z += md.imu_gyro_z-md.imu_temp_gyro_z;
	    md.imu_bias_estimation_counter++;

        md.imu_bias_estimation_timer += md.delta_time;
	    if (md.imu_bias_estimation_timer >= cd.imu_bias_estimation_wait_time)
        {
            md.imu_do_bias_estimation = false;
	        md.imu_gyro_bias_x = -md.imu_bias_estimation_gyro_sum_x / md.imu_bias_estimation_counter;
	        md.imu_gyro_bias_y = -md.imu_bias_estimation_gyro_sum_y / md.imu_bias_estimation_counter;
	        md.imu_gyro_bias_z = -md.imu_bias_estimation_gyro_sum_z / md.imu_bias_estimation_counter;
	        md.imu_bias_estimation_timer = 0;
	        md.imu_bias_estimation_counter = 0;
	        md.imu_bias_estimation_gyro_sum_x = 0;
	        md.imu_bias_estimation_gyro_sum_y = 0;
	        md.imu_bias_estimation_gyro_sum_z = 0;
	        md.imu_gravity_x = md.imu_gravity_acc_x;
	        md.imu_gravity_y = md.imu_gravity_acc_y;
	        md.imu_gravity_z = md.imu_gravity_acc_z;

            if (md.startup_sequence == SS_ImuCalibration2)
                md.startup_sequence = SS_LegCalibration2a;
        }
    }
}
