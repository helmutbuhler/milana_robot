// Here we do the IK calculation. This is needed for moving the robot to the side and also for
// arm control and for collision prevention of the arms.
// Note: This file mainly just calls the ik calculation functions defined in geometry.cpp.
// Because the ik calculation is a bit slow, we call it here in a separate thread.
// This way the result is always one frame late, but it's a good compromise to keep the mainloop fast.

#include "ik.h"
#include "../common/helper.h"
#include "main.h"
#include "../common/geometry.h"
#include <algorithm>
#include <cstring>
#include <pthread.h>
#include <semaphore.h>
#include <assert.h>

bool ik_thread_init();
void ik_thread_close();
void cancel_calculate_ik_async();
void calculate_ik_async(RobotJointState& s, const RobotIKTarget& robot_target);

bool ik_init()
{
	if (abs(1.5f) < 1.4f)
	{
		// If you include the wrong math file on Linux it can silently convert to int.
		// I have enabled warnings for this now, but just to be safe here is a runtime check.
		// Took me a while to figure this out...
		printf("wrong math.h include!\n");
		return false;
	}
	return ik_thread_init();
}

void ik_close()
{
	ik_thread_close();
}

void ik_update_()
{
	if (!cd.ik_enable || cd.servo_manual_enable)
	{
		md.leg_pos_target_ik[0] = md.common_leg_pos_target;
		md.leg_pos_target_ik[1] = md.common_leg_pos_target;
		md.hip_pos_target_ik[0] = md.common_hip_pos_target;
		md.hip_pos_target_ik[1] = md.common_hip_pos_target;
	}
	md.leaning_optimized_target_angle = 0;

	if (cd.servo_manual_enable)
	{
		for (int i = 0; i < num_arm_servos; i++)
		{
			md.arm_pos_target[i] = cd.arm_manual_target[i];
		}
		cancel_calculate_ik_async();
		return;
	}
	if (md.jump_phase != JP_None && md.jump_phase != JP_PostLanding) return;

	if (cd.arm_enable_angle_control || md.startup_sequence == SS_Hip3)
	{
		// We set the arm angle during startup because initially arm_mode is false
		// and it won't move, so lets make sure the arms are in a resonable position.
		md.ik_l_arm_a = cd.arm_angle_target[0];
		md.ik_l_arm_b = cd.arm_angle_target[1];
		md.ik_l_arm_c = cd.arm_angle_target[2];
		md.ik_r_arm_a = cd.arm_angle_target[3];
		md.ik_r_arm_b = cd.arm_angle_target[4];
		md.ik_r_arm_c = cd.arm_angle_target[5];
	}

	static int last_ik_target_counter = -1;
	if (last_ik_target_counter != cd.ik_target_counter)
	{
		last_ik_target_counter = cd.ik_target_counter;
		md.ik_enable_arm_mode = cd.ik_enable_arm_mode;
		md.ik_x_delta_target = cd.ik_x_delta_target;
		//md.ik_x_delta        = cd.ik_x_delta_target;
		md.ik_arm_target_x   = cd.ik_arm_target_x;
		md.ik_arm_target_y   = cd.ik_arm_target_y;
		md.ik_arm_target_z   = cd.ik_arm_target_z;
	}

	if (cd.ik_enable && !cd.arm_enable_angle_control && md.startup_sequence != SS_Hip3)
	{
		RobotJointState s = get_robot_joint_state(md, false);
		RobotIKTarget robot_target = get_robot_ik_target(md);
		if (cd.ik_use_thread)
			calculate_ik_async(s, robot_target);
		else
		{
			calculate_ik(s, robot_target, cd.ik_optimizer_iterations, nullptr, nullptr);
			cancel_calculate_ik_async();
		}
		md.hip_pos_target_ik[0] = (s.l.hip_angle-hip_angle_0)/cd.hip_sensor_angle_range;
		md.hip_pos_target_ik[1] = (s.r.hip_angle-hip_angle_0)/cd.hip_sensor_angle_range;
		md.leg_pos_target_ik[1] = 1 - (s.l.leg_angle-leg_angle_1)/(tau/leg_gear_reduction);
		md.leg_pos_target_ik[0] = 1 - (s.r.leg_angle-leg_angle_1)/(tau/leg_gear_reduction);
	
		md.ik_x_delta = s.x_delta;

		md.ik_l_arm_a = s.l.arm_a;
		md.ik_l_arm_b = s.l.arm_b;
		md.ik_l_arm_c = s.l.arm_c;
		if (md.ik_winking_phase == 0)
		{
			md.ik_r_arm_a = s.r.arm_a;
			md.ik_r_arm_b = s.r.arm_b;
			md.ik_r_arm_c = s.r.arm_c;
		}

		md.leaning_optimized_target_angle = s.side_angle;
	}
	else
		cancel_calculate_ik_async();

	if (cd.arm_enable_angle_control || cd.ik_enable)
	{
		// Convert angles in md.ik_x_arm_x into slider lengths.
		// Slider lengths are then linearily mapped to the final motor angles.
		float l_slider_b, l_slider_c, r_slider_b, r_slider_c;
		get_slider_lengths_from_arm_angles(
				md.ik_l_arm_b, md.ik_l_arm_c, 
				md.ik_r_arm_b, md.ik_r_arm_c,
				&l_slider_b, &l_slider_c, &r_slider_b, &r_slider_c);

		md.arm_pos_target[0] = my_map(md.ik_l_arm_a, 0, to_rad(90), cd.l_arm_a_pos_0, cd.l_arm_a_pos_90);
		md.arm_pos_target[3] = my_map(md.ik_r_arm_a, 0, to_rad(90), cd.r_arm_a_pos_0, cd.r_arm_a_pos_90);

		md.arm_pos_target[1] = my_map(l_slider_b, slider_b_length_0, slider_b_length_40, cd.l_arm_b_pos_0, cd.l_arm_b_pos_40);
		md.arm_pos_target[2] = my_map(l_slider_c, slider_c_length_0, slider_c_length_40, cd.l_arm_c_pos_0, cd.l_arm_c_pos_40);
		md.arm_pos_target[4] = my_map(r_slider_b, slider_b_length_0, slider_b_length_40, cd.r_arm_b_pos_0, cd.r_arm_b_pos_40);
		md.arm_pos_target[5] = my_map(r_slider_c, slider_c_length_0, slider_c_length_40, cd.r_arm_c_pos_0, cd.r_arm_c_pos_40);
		for (int i = 0; i < 6; i++)
			md.arm_pos_target[i] = clamp(md.arm_pos_target[i], 0, 1);
	}
}

bool ik_update()
{
	u32_micros start_time = time_micros();

	ik_update_();

	md.arm_pos_target_smooth[0] = move_towards(md.arm_pos_target_smooth[0], md.arm_pos_target[0], cd.arm_a_servo_vel_max*md.delta_time);
	md.arm_pos_target_smooth[1] = move_towards(md.arm_pos_target_smooth[1], md.arm_pos_target[1], cd.arm_b_servo_vel_max*md.delta_time);
	md.arm_pos_target_smooth[2] = move_towards(md.arm_pos_target_smooth[2], md.arm_pos_target[2], cd.arm_c_servo_vel_max*md.delta_time);
	md.arm_pos_target_smooth[3] = move_towards(md.arm_pos_target_smooth[3], md.arm_pos_target[3], cd.arm_a_servo_vel_max*md.delta_time);
	md.arm_pos_target_smooth[4] = move_towards(md.arm_pos_target_smooth[4], md.arm_pos_target[4], cd.arm_b_servo_vel_max*md.delta_time);
	md.arm_pos_target_smooth[5] = move_towards(md.arm_pos_target_smooth[5], md.arm_pos_target[5], cd.arm_c_servo_vel_max*md.delta_time);

	md.delta_time_ik = time_micros() - start_time;
	return true;
}

// If cd.ik_use_thread is set, we use the following code to do the ik calculation
// in a background thread.

static pthread_t thread_id;
static bool thread_running;
static bool ik_thread_job_pending = false;
static bool ik_thread_job_done = false;
static bool ik_thread_should_exit = false;
static sem_t ik_thread_job_sem;
static RobotJointState ik_thread_state;
static RobotIKTarget ik_thread_target;

// This function starts the ik calculation (which takes about 4ms and is quite slow)
// in a different thread and returns the result that was started in the last frame.
// This way, the ik state lags one frame behind, but doesn't slow down the main loop.
void calculate_ik_async(RobotJointState& s, const RobotIKTarget& robot_target)
{
	assert(thread_running);
	if (ik_thread_job_pending)
	{
		// If this is not the first call, we retrieve the result from the last calculation.
		// By the time this is called, the job should be long done. But, just to be safe, we busy wait.
		while (!ik_thread_job_done)
		{
			//printf("ik sleep!\n");
			precise_sleep(0.001);
		}
		s = ik_thread_state;
		// Note that we ignore the state passed via parameter in this case
		// but that is fine because all the current information we need is in robot_target.
	}

	// Trigger calculation in thread.
	ik_thread_job_pending = true;
	ik_thread_job_done = false;
	ik_thread_state = s;
	ik_thread_target = robot_target;
	sem_post(&ik_thread_job_sem);
}

// If we called calculate_ik_async and the next frame we don't actually need the result we call this.
// Otherwise, if we call calculate_ik_async later again, we get the old result, and we don't want that.
void cancel_calculate_ik_async()
{
	ik_thread_job_pending = false;
}

void* ik_thread(void*)
{
	while (true)
	{
		// Wait for next job
		int r = sem_wait(&ik_thread_job_sem);
		if (r)
		{
			printf("sem_wait error! %d\n", r);
			return nullptr;
		}

		if (ik_thread_should_exit)
			break;
		
		calculate_ik(ik_thread_state, ik_thread_target, cd.ik_optimizer_iterations, nullptr, nullptr);
		ik_thread_job_done = true;
	}
	return nullptr;
}

bool ik_thread_init()
{
    // Initialize the semaphore with a value of 0.
    // Note the second argument: passing zero denotes
    // that the semaphore is shared between threads (and
    // not processes).
    if (sem_init(&ik_thread_job_sem, 0, 0))
    {
        printf("Could not initialize a semaphore\n");
        return false;
    }
	ik_thread_job_pending = false;
	ik_thread_job_done = false;
	ik_thread_should_exit = false;
	int r = pthread_create(&thread_id, nullptr, ik_thread, nullptr);
	if (r != 0)
		return false;
	thread_running = true;
	return true;
}

void ik_thread_close()
{
	if (!thread_running)
		return;
	ik_thread_should_exit = true;
	sem_post(&ik_thread_job_sem);
	pthread_join(thread_id, nullptr);
	thread_running = false;
    sem_destroy(&ik_thread_job_sem);
}
