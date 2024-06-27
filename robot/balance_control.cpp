// This is in essence the pid controller that processes the imu x_angle, the target velocity and the
// measured velocity in the wheels and calculates the target velocity for the wheels.
// The target velocity is then controlled in the ODrive for the wheels.

#include "balance_control.h"
#include "main.h"
#include "../common/geometry.h"
#include <algorithm>
#include <math.h>
#include <string>

bool balance_control_init()
{
	return true;
}

void balance_control_close()
{
}

/*
There are different velocities involved here:
md.balance_control_vel_user:         The velocity wanted by the user.
md.balance_control_vel_quasi:        The processed velocity, that makes sure the long-term velocity is the target velocity.
md.balance_control_vel_output:       The processed velocity, that makes sure the robot remains stable short-term.
md.balance_control_vel_quasi_smooth: Smooth version of vel_quasi. If this value differs too much from vel_output, the robot has likely crashed or was picked up.
*/

void balance_control_update_squat_angle_and_coi()
{
	float delta_time = md.delta_time;

	float leg_pos_l =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
	float leg_pos_r = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
	float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
	float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;
	float hip_pos_l = md.hip_sensor_pos[0]-md.hip_balance_current;
	float hip_pos_r = md.hip_sensor_pos[1]-md.hip_balance_current;
	float hip_angle_l = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
	float hip_angle_r = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[1];
	float foot_pos_l = -md.foot_pos[1];
	float foot_pos_r =  md.foot_pos[0];

	{
		float squat_length_l, squat_length_r;
		float target_angle_l = calculate_squat_angle(hip_pos_l, leg_pos_l, &squat_length_l);
		float target_angle_r = calculate_squat_angle(hip_pos_r, leg_pos_r, &squat_length_r);
		float old_target_angle = md.balance_control_target_angle;
		md.balance_control_target_angle = (target_angle_l + target_angle_r) * 0.5f;
		md.balance_control_squat_length = (squat_length_l + squat_length_r) * 0.5f;

		md.balance_control_target_angle_vel = update_smooth(
				md.balance_control_target_angle_vel,
				(md.balance_control_target_angle-old_target_angle) / delta_time,
				cd.balance_control_target_angle_vel_smooth, cd.target_delta_time_ms);
	}

	// Calculate acceleration of coi in two ways:
	// With double differentiation of measured position (coi_world_acc)
	// With calculated acceleration of measured angle (coi_theta_acc) 
	{

		// Retrieve theta from a few frames before
		float theta;
		{
			const int max_history = 16;
			static float error_p_history[max_history];
			static int error_p_history_pos = 0;
			error_p_history_pos = (error_p_history_pos+1)%max_history;
			error_p_history[error_p_history_pos] = md.error_p + md.t_correction_angle - md.t_correction_angle_raw;
			theta = error_p_history[(error_p_history_pos+max_history-cd.coi_theta_past)%max_history];
		}
		
		float coi_x;
		get_com_world_position(md.imu_x_angle, theta,
				hip_angle_l, leg_angle_l, foot_pos_l,
				hip_angle_r, leg_angle_r, foot_pos_r,
				0, 0,
				&coi_x, 0,
				&md.coi_theta_acc);
		
		float old_coi_world_pos = md.coi_world_pos;
		md.coi_world_pos = coi_x;

		float old_coi_world_vel = md.coi_world_vel;
		md.coi_world_vel = update_smooth(
				old_coi_world_vel,
				(md.coi_world_pos-old_coi_world_pos) / delta_time,
				cd.coi_world_vel_smooth_factor, cd.target_delta_time_ms);

		float old_coi_world_acc = md.coi_world_acc;
		md.coi_world_acc = update_smooth(
				old_coi_world_acc,
				(md.coi_world_vel-old_coi_world_vel) / delta_time / 1000 / earth_acc,
				cd.coi_world_acc_smooth_factor, cd.target_delta_time_ms);
	}
}

bool balance_control_update()
{
	// In case there is a delta time spike, it's better to treat dt as constant in some parts of this function.
	float delta_time = cd.max_delta_time_ms * 0.001f;

	balance_control_update_squat_angle_and_coi();

	if (cd.balance_control_enable && md.is_standing)
	{
		md.balance_control_vel_user = move_towards(
				md.balance_control_vel_user,
				cd.balance_control_vel_target,
				cd.balance_control_vel_user_acceleration*delta_time);

		md.balance_control_vel_quasi = md.balance_control_vel_user*md.balance_control_vel_extra_factor;

		float max_vel = (cd.balance_control_vel_user_max+1.0f) * md.balance_control_vel_extra_factor;
		md.balance_control_vel_quasi = clamp(md.balance_control_vel_quasi, -max_vel, max_vel);
		
		if (cd.balance_control_enable_position_control || md.ik_enable_arm_mode)
		{
			// In arm mode, IK determines the robotposition and there is no good way
			// to handle that in velocity mode.
			md.balance_control_position_control_target += md.balance_control_vel_quasi * md.delta_time;

			float pos_target = md.balance_control_position_control_target + md.ik_x_delta/(tau*wheel_radius);
			float position_control_error   = pos_target - md.coi_world_pos/(tau*wheel_radius);
			float position_control_error_d = 0          - md.coi_world_vel/(tau*wheel_radius);

			float position_control_gain_p = clamp_and_map(md.balance_control_squat_length, 80, 160, cd.balance_control_position_control_gain_p_low, cd.balance_control_position_control_gain_p_high);
			float position_control_gain_d = clamp_and_map(md.balance_control_squat_length, 80, 160, cd.balance_control_position_control_gain_d_low, cd.balance_control_position_control_gain_d_high);

			md.balance_control_vel_quasi =  position_control_error   * position_control_gain_p;
			md.balance_control_vel_quasi += position_control_error_d * position_control_gain_d;
		}
		else
		{
			md.balance_control_position_control_target = md.coi_world_pos/(tau*wheel_radius) - md.ik_x_delta/(tau*wheel_radius);
		}

		md.balance_control_vel_quasi = clamp(md.balance_control_vel_quasi, -max_vel, max_vel);
		
		{
			float new_smooth = update_smooth(
					md.balance_control_vel_quasi_smooth,
					md.balance_control_vel_quasi,
					cd.balance_control_vel_quasi_smooth_factor, cd.target_delta_time_ms);
			md.balance_control_vel_quasi_smooth = clamp(
					new_smooth,
					md.balance_control_vel_quasi_smooth-cd.balance_control_vel_quasi_smooth_max_acc*delta_time,
					md.balance_control_vel_quasi_smooth+cd.balance_control_vel_quasi_smooth_max_acc*delta_time);
		}

		md.balance_control_rotation_vel = move_towards(
				md.balance_control_rotation_vel,
				cd.balance_control_rotation_vel_user_target,
				cd.balance_control_rotation_vel_acc * delta_time);
		if (cd.leaning_clamp_rotation_vel)
		{
			// When the robot moves very fast while rotating, we run the risk of falling
			// to the side due to centrifugal force.
			// To prevent this, the leaning code calculates the safe bounds of our
			// rotation velocity depending on the current side angle, speed and height of the
			// center of mass.
			md.balance_control_rotation_vel = clamp(
					md.balance_control_rotation_vel,
					md.leaning_min_rotation_vel,
					md.leaning_max_rotation_vel);
		}
	}

	float old_error_p = md.error_p;
	md.error_p = md.imu_x_angle - md.balance_control_target_angle - md.t_correction_angle;
	md.avg_error_p = md.avg_error_p * (1-cd.stepper_avg_smooth) + md.error_p * cd.stepper_avg_smooth;
	
	md.error_d = update_smooth(
			md.error_d,
			(md.error_p-old_error_p) / delta_time,
			cd.balance_control_error_d_smooth, cd.target_delta_time_ms);

	if (md.startup_sequence == SS_StartBalancing1)
	{
		if (abs(md.leg_rel_angle[0]-md.leg_rel_angle_target[0]) > 0.1f ||
			abs(md.leg_rel_angle[1]-md.leg_rel_angle_target[1]) > 0.1f)
		{
			// Before we balance, we check that the leg joints are where they are supposed to be.
			// Also see "monitor_log_startup_wrong_leg_position.bin"
			printf("startup sequence: leg joint is not in position. (one leg encoder might be broken)\n");
			md.startup_sequence = SS_Failed;
		}
		else
		{
			md.startup_sequence = SS_StartBalancing2;
			md.startup_sequence_timer = 0;
		}
	}
	if (md.startup_sequence == SS_StartBalancing2)
	{
		// If we were not able to start balancing within a timelimit, there
		// might be something wrong. Let's play it save and cancel the startup sequence.
		if (md.startup_sequence_timer > 0.5f)
		{
			printf("startup sequence: start balancing failed!\n");
			md.startup_sequence = SS_Failed;
		}
	}

	float world_angle = 0;
	world_angle += md.foot_pos[0];
	world_angle += md.foot_pos[1]; // (double negated)
	world_angle *= tau * wheel_radius / cd.wheel_side_distance;
	float world_angle_delta = world_angle-md.balance_control_world_angle;
	md.balance_control_world_angle = world_angle;

	md.balance_control_gain_p 				= cd.balance_control_gain_p;
	md.balance_control_gain_i 				= cd.balance_control_gain_i;
	md.balance_control_gain_d 				= cd.balance_control_gain_d;
	md.balance_control_gain_vel_high        = cd.balance_control_gain_vel_high;
	md.balance_control_gain_vel_low		    = cd.balance_control_gain_vel_low;
	md.balance_control_gain_backcalculation = cd.balance_control_gain_backcalculation;
	md.balance_control_force_still          = cd.balance_control_force_still;

	bool balance_control_enable = cd.balance_control_enable;
	if (md.startup_sequence != SS_None && md.startup_sequence < SS_StartBalancing1)
		balance_control_enable = false;
	if (balance_control_enable)
	{
		bool stop_motors = false;
		if (md.jump_phase == JP_Jumping)
			stop_motors = true;
		if (!stop_motors || cd.balance_control_force_still)
		{	
			md.vel_p = md.error_p * cd.balance_control_gain_p;
			md.vel_d = md.error_d * cd.balance_control_gain_d;
			{
				// We clamp theta that gets added to integrator.
				// This stabilizes recovery in extreme balance situations.
				float theta = md.error_p;
				theta = clamp(theta, -cd.balance_control_max_theta_integrator, cd.balance_control_max_theta_integrator);
				md.balance_control_integrator += theta * delta_time * cd.balance_control_gain_i;
			}

			md.vel_pos = 0;
			{
				md.vel_pos += clamp(cd.balance_control_use_coi_vel ? md.coi_world_vel/(tau*wheel_radius) : md.balance_control_vel_output, -cd.foot_control_vel_limit, cd.foot_control_vel_limit);
				md.vel_pos -= md.balance_control_vel_quasi;

				float balance_control_gain_vel = clamp_and_map(md.balance_control_squat_length, 80, 160, cd.balance_control_gain_vel_low, cd.balance_control_gain_vel_high);
				if (md.jump_phase != JP_Positioning && cd.balance_control_gain_vel_landing_warmup_time != 0)
					balance_control_gain_vel = lerp(0, balance_control_gain_vel, std::min(md.post_landing_timer / cd.balance_control_gain_vel_landing_warmup_time, 1.0f));
				md.vel_pos *= balance_control_gain_vel * delta_time;

				// Clamping this error effectivly limits the acceleration of target velocity.
				// This is important in case balance_control_vel_quasi changes rapidly.
				md.vel_pos = clamp(md.vel_pos, -cd.balance_control_vel_acceleration_limit, cd.balance_control_vel_acceleration_limit);

				md.balance_control_integrator += md.vel_pos;
			}
			
			// backcalculation
			float vel_error_l =  md.foot_vel_coarse[0]-(md.balance_control_vel_output+md.balance_control_delta_vel_l-md.balance_control_rotation_error*cd.balance_control_rotation_error_factor);
			float vel_error_r = -md.foot_vel_coarse[1]-(md.balance_control_vel_output+md.balance_control_delta_vel_r+md.balance_control_rotation_error*cd.balance_control_rotation_error_factor);
			md.vel_backcalculation = (vel_error_l+vel_error_r) * 0.5f * delta_time * cd.balance_control_gain_backcalculation;
			md.balance_control_integrator += md.vel_backcalculation;
			
			// Tiny gain that improves stability during hip or leg movement
			md.balance_control_integrator += md.balance_control_target_angle_vel*cd.balance_control_target_angle_vel_gain;

			// hip balance integrator (unused currently)
			md.balance_control_integrator += md.hip_balance_current * cd.hip_balance_integrator;
			
			if (md.jump_phase == JP_Positioning)
				md.balance_control_integrator += cd.positioning_vel_delta;

			md.balance_control_vel_output = md.vel_p + md.vel_d + md.balance_control_integrator;

			if (cd.balance_control_force_still)
			{
				md.balance_control_vel_output = 0;
				//md.post_landing_timer = 0; // uncomment this to test balancing with extreme theta
				md.balance_control_integrator = 0;
			}
			else if (md.jump_phase == JP_LegUp || md.jump_phase == JP_Falling || md.post_landing_timer < cd.landing_foot_vel_delta_time)
			{
				// During the jump until a short time after landing, we do this:
				//  - Disable Integrator to prevent windup.
				//  - Add a constant velocity. This makes it easier for the leg to close and
				//    has the effect that the kinetic energy is absorbed in the leg motors.
				//    Otherwise the body of the robot starts to rotate and we run into balancing problems.
				md.balance_control_vel_output -= md.balance_control_integrator;
				md.balance_control_integrator = cd.landing_foot_vel_delta+md.balance_control_vel_user;
				md.balance_control_vel_output += md.balance_control_integrator;
			}
			else if (abs(md.balance_control_rotation_vel) < cd.t_correction_max_rotation_vel)
			{
				float t_correction_error = md.balance_control_vel_output-md.balance_control_vel_quasi;
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


			if (md.startup_sequence == SS_StartBalancing2)
			{
				md.is_standing = abs(md.error_p) < cd.startup_balance_control_standing_tolerance;
				if (abs(md.error_p) < cd.balance_control_standing_theta_tolerance * 0.1f)
					md.startup_sequence = SS_FinalLegHipMovement;
			}
			else if (md.is_standing)
			{
				if (abs(md.error_p) < cd.balance_control_standing_theta_tolerance)
				{
					// If we are able to recover in extreme balance situations error_p will switch the sign.
					// In that case we reset the timer to give us some time.
					// See monitor_log_leaning05 history_plot_pos = 1370
					// See monitor_log_arm1 history_plot_pos = 1000 where we do not want to reset to zero, but approarch zero so a very high delta can still increase the timer.
					md.balance_control_standing_action_tolerance_timer *= 0.1f;
				}
				float delta = abs(md.balance_control_vel_output-md.balance_control_vel_quasi_smooth);
				if (delta > cd.balance_control_standing_action_tolerance)
				{
					md.balance_control_standing_action_tolerance_timer +=
							(delta - cd.balance_control_standing_action_tolerance) * md.delta_time;
					if (md.balance_control_standing_action_tolerance_timer > cd.balance_control_standing_action_tolerance_time)
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
				md.is_standing = abs(md.error_p) < cd.balance_control_standing_theta_tolerance * 0.1f;
				//md.is_standing = abs(md.error_p) < cd.stepper_standing_tolerance;
			}
			

			if (md.oscilloscope_state == 3 && md.oscilloscope_start != md.oscilloscope_end)
			{
				// Oscilloscope recording is being transmitted.
				// This is so slow that balancing makes no sense
				md.is_standing = false;
			}

			if (cd.balance_control_integrator_rotation_decay)
			{
				// Fast rotation decreases our speed.
				// Reflecting this in the integrator stabilizes the robot.
				md.balance_control_integrator *= cos(world_angle_delta);
			}
		}
		else
		{
			// We are in jumping mode, we keep the motors fixed.
			md.balance_control_vel_output = 0;
			md.is_standing = true;
			md.vel_p = 0;
			md.vel_d = 0;
			md.balance_control_integrator = 0;
			md.vel_pos = 0;
			//md.t_correction_angle = 0;
		}
	}
	else
	{
		md.is_standing = false;
	}

	if (!md.is_standing)
	{
		md.vel_p = 0;
		md.vel_d = 0;
		md.balance_control_integrator = 0;
		md.vel_pos = 0;
		md.t_correction_angle = 0;
		md.t_correction_angle_raw = 0;
		md.balance_control_vel_output = 0;
		md.balance_control_vel_user = 0;
		md.balance_control_vel_quasi = 0;
		md.balance_control_vel_quasi_smooth = 0;
		
		md.balance_control_position_control_target = md.coi_world_pos/(tau*wheel_radius) - md.ik_x_delta/(tau*wheel_radius);

		md.balance_control_rotation_error_bias = -md.foot_pos[0] - md.foot_pos[1];
	}

	{
		float leg_pos_l =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
		float leg_pos_r = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
		float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
		float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;
		float hip_angle_l = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
		float hip_angle_r = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[1];

		// If the angle between the lower leg and the gravity vector changes, we need to compensate
		// the resulting rotation:
		float delta_pos_lower_leg_angle_l = -(md.imu_x_angle + hip_angle_l + leg_angle_l) / tau;
		float delta_pos_lower_leg_angle_r = -(md.imu_x_angle + hip_angle_r + leg_angle_r) / tau;

		// Also, when one foot is moved in front of the other, we need to rotate the corresponding wheel
		// the same amount in that direction. Otherwise the robot would start to turn.
		// This is especially important when the robot moves with one leg over a ramp.
		float delta_l = leg_delta_from_angles(md.imu_x_angle, hip_angle_l, leg_angle_l);
		float delta_r = leg_delta_from_angles(md.imu_x_angle, hip_angle_r, leg_angle_r);
		float delta = delta_l-delta_r;
		float delta_pos_foot_pos_l =  delta*0.5f/(tau*wheel_radius);
		float delta_pos_foot_pos_r = -delta*0.5f/(tau*wheel_radius);

		// Combine both corrections and calculate velocity
		float old_balance_control_delta_pos_l = md.balance_control_delta_pos_l;
		float old_balance_control_delta_pos_r = md.balance_control_delta_pos_r;
		md.balance_control_delta_pos_l = delta_pos_lower_leg_angle_l + delta_pos_foot_pos_l;
		md.balance_control_delta_pos_r = delta_pos_lower_leg_angle_r + delta_pos_foot_pos_r;
		md.balance_control_delta_vel_l = (md.balance_control_delta_pos_l - old_balance_control_delta_pos_l) / delta_time;
		md.balance_control_delta_vel_r = (md.balance_control_delta_pos_r - old_balance_control_delta_pos_r) / delta_time;

		md.balance_control_rotation_error_bias += md.balance_control_rotation_vel * 2 * delta_time;
		md.balance_control_rotation_error = 0;
		md.balance_control_rotation_error += md.foot_pos[0];
		md.balance_control_rotation_error += md.foot_pos[1]; // (double negated)
		md.balance_control_rotation_error += md.balance_control_rotation_error_bias;
		md.balance_control_rotation_error -= md.balance_control_delta_pos_l;
		md.balance_control_rotation_error += md.balance_control_delta_pos_r;
	}

	return true;
}
