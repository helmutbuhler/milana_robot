// The Jetson Nano is connected to a PCA9685 servo controller. This servo controller controls
// both hip servos and also all the servos for the arms. Here we handle communication with the PCA9685.
// 
// Calibrating the hip motors is a bit tricky. We use servos for them, but the analog sensors in those
// servos are too unreliable. We use extra incremental sensors for precise data (we get that data with
// help from the leg odrive). Unfortunately those sensors are not absolute, they need to find the index
// on every startup. During the startupsequence there is an algo implemented here that calibrates them,
// but it's all a bit messy.
//
// The arm servos only need to be calibrated once manually. We don't have any feedback on those and just
// assume that they are where we command them to go.

// converted from PCA9685.py in:
// https://github.com/adafruit/Adafruit_Python_PCA9685
// Original Author: Tony DiCola
#include "servo.h"
#include "../common/common.h"
#include "main.h"
#include "../common/geometry.h"
#include "leg_control.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>
#include <algorithm>
#include <assert.h>
#include <string.h>


#define PCA9685_ADDRESS    0x40
#define I2C_ADDRESS        "/dev/i2c-1"
#define MODE1              0x00
#define MODE2              0x01
#define SUBADR1            0x02
#define SUBADR2            0x03
#define SUBADR3            0x04
#define PRESCALE           0xFE
#define LED0_ON_L          0x06
#define LED0_ON_H          0x07
#define LED0_OFF_L         0x08
#define LED0_OFF_H         0x09
#define ALL_LED_ON_L       0xFA
#define ALL_LED_ON_H       0xFB
#define ALL_LED_OFF_L      0xFC
#define ALL_LED_OFF_H      0xFD

#define RESTART            0x80
#define SLEEP              0x10
#define ALLCALL            0x01
#define INVRT              0x10
#define OUTDRV             0x04


#define WRITE_1(a) \
	buffer[0] = (u8)(a); \
	if (write(servo_file, buffer, 1) != 1) \
		return false;
#define WRITE_2(a, b) \
	buffer[0] = (u8)(a); \
	buffer[1] = (u8)(b); \
	if (write(servo_file, buffer, 2) != 2) \
		return false;

#define WRITE_READ_1(a) \
	WRITE_1(a); \
	if (read(servo_file, buffer, 1) != 1) \
		return false;

static int servo_file = -1;
static u8 buffer[2];
static bool is_servo_enabled;
static u16 channel_pwm[16];


bool set_pwm_freq(double freq_hz);
bool set_pwm(u8 channel, u16 on, u16 off);
bool set_all_pwm(u16 on, u16 off);
bool set_servos_enabled(bool enable);

bool init_servo_motors_helper()
{
    //Initialize the PCA9685.
	servo_file = open(I2C_ADDRESS, O_RDWR);
	if (servo_file == -1)
		return false;
    if (ioctl(servo_file, I2C_SLAVE, PCA9685_ADDRESS) < 0)
        return false;

	memset(channel_pwm, 0, sizeof(channel_pwm));
    if (!set_all_pwm(0, 0))
		return false;
    WRITE_2(MODE2, OUTDRV);
    WRITE_2(MODE1, ALLCALL);
    precise_sleep(0.005); // wait for oscillator
	if (!set_servos_enabled(true))
		return false;
    precise_sleep(0.005); // wait for oscillator
	
	set_pwm_freq(60);

	return true;
}

bool servo_motors_init()
{
	if (!init_servo_motors_helper())
	{
		fprintf(stderr, "Failed to initialize servo motors\n");
		servo_motors_close();
		return false;
	}
	return true;
}

void servo_motors_close()
{
	if (servo_file == -1)
		return;
	set_servos_enabled(false);
	close(servo_file);
	servo_file = -1;
}

bool set_pwm_freq(double freq_hz)
{
    // Set the PWM frequency to the provided value in hertz.
    double prescaleval = 25000000.0;    // 25MHz
    prescaleval /= 4096.0;       // 12-bit
    prescaleval /= freq_hz;
    prescaleval -= 1.0;
    //fprintf(stdout, "Setting PWM frequency to %f Hz\n", freq_hz);
    //fprintf(stdout, "Estimated pre-scale: %f\n", prescaleval);
    int prescale = int(prescaleval + 0.5);
    //fprintf(stdout, "Final pre-scale: %i\n", prescale);

	WRITE_READ_1(MODE1);
	u8 oldmode = buffer[0];
    u8 newmode = (oldmode & 0x7F) | SLEEP;
    WRITE_2(MODE1, newmode);  // go to sleep
    WRITE_2(PRESCALE, (u8)prescale);
    WRITE_2(MODE1, oldmode);
    precise_sleep(0.005);
    WRITE_2(MODE1, oldmode | RESTART);
	return true;
}

bool set_pwm(u8 channel, u16 on, u16 off)
{
	if (off == channel_pwm[channel])
		return true;
	channel_pwm[channel] = off;
	assert(on == 0);

	// Sets a single PWM channel.
    //WRITE_2(LED0_ON_L+4*channel, on & 0xFF);
    //WRITE_2(LED0_ON_H+4*channel, on >> 8);
    WRITE_2(LED0_OFF_L+4*channel, off & 0xFF);
    WRITE_2(LED0_OFF_H+4*channel, off >> 8);
	return true;
}

bool set_all_pwm(u16 on, u16 off)
{
    // Sets all PWM channels.
    WRITE_2(ALL_LED_ON_L, on & 0xFF);
    WRITE_2(ALL_LED_ON_H, on >> 8);
    WRITE_2(ALL_LED_OFF_L, off & 0xFF);
    WRITE_2(ALL_LED_OFF_H, off >> 8);
	return true;
}

bool set_servos_enabled(bool enable)
{
	if (!enable)
	{
		// Disable each used channel first here.
		// Otherwise the chip randomly sets new frequencies, which
		// causes the servos to move unexpectedly.
		// Happens only with 10% chance and I don't know why, but this
		// seems to fix it.
		for (int i = 0; i < 16; i++)
			if (channel_pwm[i])
			{
				set_pwm(i, 0, 0x1000);
				channel_pwm[i] = 0;
			}
		precise_sleep(0.005); // wait for oscillator
	}
	WRITE_READ_1(MODE1);
	u8 mode = buffer[0];
	if (enable)
	    mode = mode & (u8)(~SLEEP);
	else
	    mode = mode | SLEEP;
    WRITE_2(MODE1, mode);
	is_servo_enabled = enable;
	return true;
}

bool servo_motors_update_()
{
	bool should_enable = cd.servo_manual_enable || cd.servo_enable || cd.arm_enable_angle_control;
	if (md.startup_sequence != SS_None && md.startup_sequence < SS_Hip1)
		should_enable = false;
	if (is_servo_enabled != should_enable)
	{
		if (!set_servos_enabled(should_enable))
		{
			perror("Servos");
			return false;
		}
	}

	if (!is_servo_enabled)
		return true;

	{
		// In extreme balance situations, this code tries to help additionally with the hip.
		// It's not really working yet though...
		md.hip_balance_target = 0;
		md.hip_balance_target -= (md.balance_control_vel_quasi_smooth-md.balance_control_vel_output_smooth) * cd.hip_balance_action_factor;
		md.hip_balance_target -= md.vel_backcalculation * cd.hip_balance_backcalc_factor;
		md.hip_balance_target += md.error_p * cd.hip_balance_theta_factor;
		md.hip_balance_target += (md.balance_control_vel_output - md.balance_control_vel_quasi) * cd.hip_balance_x_factor;
		md.hip_balance_target -= clamp(md.hip_balance_target, -cd.hip_balance_deadzone, cd.hip_balance_deadzone);
		md.hip_balance_target *= cd.hip_balance_factor;
		md.hip_balance_current = move_towards(md.hip_balance_current, md.hip_balance_target, cd.servo_vel_max*md.delta_time);
		md.hip_balance_current = clamp(md.hip_balance_current, 0-md.common_hip_pos_target, 1-md.common_hip_pos_target);
	}
	
	if (cd.servo_manual_enable)
	{
		md.common_servo_vel = 0;
		md.hip_pos_target[0] = cd.servo_manual_target[0];
		md.hip_pos_target[1] = cd.servo_manual_target[1];
	}
	else if (md.jump_phase == JP_Falling)
	{
		md.common_servo_vel = 0;
		//md.common_servo_vel = md.error_p*cd.fall_servo_correction;
		//md.common_servo_vel = cd.fall_servo_target-md.common_servo;
		//if (md.x_angle < cd.fall_servo_x_angle_stop)
		//	md.common_hip_pos_target = move_towards(md.common_hip_pos_target, cd.fall_servo_target, cd.servo_vel_max*md.delta_time);
		md.hip_pos_target[0] = md.common_hip_pos_target;
		md.hip_pos_target[1] = md.common_hip_pos_target;

#if 1
		// if the robot rotates to one side while falling, try to rotate the hips such that the wheels collide even with the ground.
		// This is probably not needed anymore, but it doesn't hurt.
		float common_servo_angle = lerp(hip_angle_0, hip_angle_0+cd.hip_sensor_angle_range, md.common_hip_pos_target);
		float leg_right_y = leg_height_from_angles_falling(md.imu_x_angle, common_servo_angle, md.imu_side_angle, -1);
		float leg_left_y  = leg_height_from_angles_falling(md.imu_x_angle, common_servo_angle, md.imu_side_angle, 1);
		{
			float hip_angle = common_servo_angle;
			bool adjust_left = md.imu_side_angle > 0;

			// Little optimization to find out optimal hip angles.
			{
				float best_error = -1;
				float steps[] = {0, 0.2f, -0.2f, 0.1f, -0.1f, 0.05f, -0.05f, 0.02f, -0.02f, 0.01f, -0.01f, 0.005f, -0.005f, 0.002f, -0.002f, 0.001f, -0.001f};
				for (int i = 0; i < 17; i++)
				{
					try_again:
					if (adjust_left)
						leg_left_y = leg_height_from_angles_falling(md.imu_x_angle, hip_angle+steps[i], md.imu_side_angle, 1);
					else
						leg_right_y = leg_height_from_angles_falling(md.imu_x_angle, hip_angle+steps[i], md.imu_side_angle, -1);
					float error = sq(leg_left_y-leg_right_y);
					if (best_error < 0 || error < best_error)
					{
						best_error = error;
						hip_angle += steps[i];
						goto try_again;
					}
				}
			}
			md.hip_pos_target[adjust_left ? 0 : 1] = (hip_angle-hip_angle_0) / cd.hip_sensor_angle_range;
			md.hip_pos_target[adjust_left ? 0 : 1] = clamp(md.hip_pos_target[adjust_left ? 0 : 1], 0, 1);
		}
#endif
		//md.hip_pos_target[0] = md.common_hip_pos_target;
	}
	else
	{
		md.hip_pos_target[0] = md.hip_pos_target_ik[0]+md.hip_balance_current;
		md.hip_pos_target[1] = md.hip_pos_target_ik[1]+md.hip_balance_current;
	}

	static int initial_hip_sensor_index_count[2];
	if (md.startup_sequence == SS_Hip1)
	{
		if (md.common_hip_pos_target < cd.startup_hip1_servo_min &&
			(md.hip_sensor_vel[0]+md.hip_sensor_vel[1])*0.5f < cd.startup_hip1_servo_vel_threshold)
		{
			md.startup_sequence = SS_Hip2;
			initial_hip_sensor_index_count[0] = md.hip_sensor_index_count[0];
			initial_hip_sensor_index_count[1] = md.hip_sensor_index_count[1];
		}
		else
		{
			md.common_hip_pos_target += cd.startup_hip1_servo_vel*md.delta_time;
			if (md.common_hip_pos_target <= 0)
			{
				md.common_hip_pos_target = 0;
				md.startup_sequence = SS_Failed;
				printf("startup sequence: servo calibration failed!\n");
			}
			md.hip_pos_target[0] = md.common_hip_pos_target;
			md.hip_pos_target[1] = md.common_hip_pos_target;
		}
	}
	static int initial_hip_sensor_index_error[2];
	if (md.startup_sequence == SS_Hip2)
	{
		static float hip_sensor_max0;
		static float hip_sensor_max1;
		static int final_counter = 0;
		if (md.common_hip_pos_target == 1)
		{
			hip_sensor_max0 = std::max(hip_sensor_max0, md.hip_sensor_pos_relative[0]);
			hip_sensor_max1 = std::max(hip_sensor_max1, md.hip_sensor_pos_relative[1]);

			final_counter++;
			if (final_counter > 60)
			{
				// Check each hip index counted at least once.
				if (initial_hip_sensor_index_count[0] == md.hip_sensor_index_count[0] ||
					initial_hip_sensor_index_count[1] == md.hip_sensor_index_count[1])
				{
					printf("hip sensor index not found during startup!\n");
					md.startup_sequence = SS_Failed;
				}
				else
				{
					md.startup_sequence = SS_ImuCalibration1;
					md.hip_sensor_pos_base[0] = 1-hip_sensor_max0;
					md.hip_sensor_pos_base[1] = 1-hip_sensor_max1;

					// The index is found now, so the index error shouldn't change anymore.
					// We save the current value now and later check during phase SS_Hip3
					// that it stays the same.
					initial_hip_sensor_index_error[0] = md.hip_sensor_index_error[0];
					initial_hip_sensor_index_error[1] = md.hip_sensor_index_error[1];
				}
			}
		}
		else
		{
			final_counter = 0;

			// Reset max value when starting this calibration phase.
			// This is neccessary because the encoder sleeve can slip in some cases.
			// If so, we want to be able to correct that with running the startup sequence again.
			hip_sensor_max0 = md.hip_sensor_pos_relative[0];
			hip_sensor_max1 = md.hip_sensor_pos_relative[1];
		}
		md.common_hip_pos_target += cd.startup_hip2_servo_vel*md.delta_time;
		if (md.common_hip_pos_target >= 1)
		{
			md.common_hip_pos_target = 1;
		}
		md.hip_pos_target[0] = md.common_hip_pos_target;
		md.hip_pos_target[1] = md.common_hip_pos_target;
	}
	if (md.startup_sequence == SS_Hip3)
	{
		const float common_hip_pos_final_target = 0.2f;
		md.common_hip_pos_target -= cd.startup_hip3_servo_vel*md.delta_time;
		if (md.common_hip_pos_target <= common_hip_pos_final_target)
		{
			md.common_hip_pos_target = common_hip_pos_final_target;
			md.startup_sequence = SS_FootIndex1;
			stop_hip_sensor_index();
		}
		md.hip_pos_target[0] = md.common_hip_pos_target;
		md.hip_pos_target[1] = md.common_hip_pos_target;

		if (initial_hip_sensor_index_error[0] != md.hip_sensor_index_error[0] ||
			initial_hip_sensor_index_error[1] != md.hip_sensor_index_error[1])
		{
			// The index changed since we captured it in phase SS_Hip2.
			// That means the encoder is defective or EM noise disrupts its signal.
			printf("hip sensor sends corrupt data during startup!\n");
			md.startup_sequence = SS_Failed;
		}
	}
	if (md.startup_sequence == SS_FinalLegHipMovement)
	{
		const float common_hip_pos_final_target = cd.startup_hip_final_pos;
		md.common_hip_pos_target += cd.startup_hip3_servo_vel*md.delta_time;
		if (md.common_hip_pos_target >= common_hip_pos_final_target)
		{
			md.common_hip_pos_target = common_hip_pos_final_target;
		}
		if (md.common_hip_pos_target == common_hip_pos_final_target &&
			md.common_leg_pos_target == cd.startup_leg_final_pos)
		{
			md.common_hip_pos_target = common_hip_pos_final_target;
			md.startup_sequence = SS_None;
		}
		md.hip_pos_target[0] = md.common_hip_pos_target;
		md.hip_pos_target[1] = md.common_hip_pos_target;
	}

	md.servo_pos[0] = move_towards(md.servo_pos[0], md.hip_pos_target[0], cd.servo_vel_max*md.delta_time);
	md.servo_pos[1] = move_towards(md.servo_pos[1], md.hip_pos_target[1], cd.servo_vel_max*md.delta_time);

	if (cd.hip_sensor_enable_correction &&
		(md.startup_sequence == SS_None ||
		md.startup_sequence >= SS_Hip3))
	{
		static float last_imu_side_angle;
		float delta_side_angle = (md.imu_side_angle - last_imu_side_angle) * cd.hip_sensor_correction_side_angle_factor;
		last_imu_side_angle = md.imu_side_angle;
		for (int i = 0; i < 2; i++)
		{
			float delta = md.servo_pos[i] - md.hip_sensor_pos[i];
			if (abs(delta) < cd.hip_sensor_correction_dead_zone)
				delta = 0;
			else
				delta *= cd.hip_sensor_correction_factor * md.delta_time;
			md.hip_sensor_correction_delta[i] += delta;
			md.hip_sensor_correction_delta[i] += delta_side_angle * (i == 0 ? -1 : 1);
			float target = md.hip_pos_target[i] + md.hip_sensor_correction_delta[i];
			md.hip_sensor_correction_delta[i] = clamp(target, 0, 1) - md.hip_pos_target[i];
			md.hip_sensor_correction_delta[i] = clamp(md.hip_sensor_correction_delta[i], -cd.hip_sensor_max_deviation, cd.hip_sensor_max_deviation);
		}
	}
	else
	{
		md.hip_sensor_correction_delta[0] = 0;
		md.hip_sensor_correction_delta[1] = 0;
	}

	if (is_servo_enabled)
	{
		// hip servos
		for (int i = 0; i < 2; i++)
		{
			if (cd.servo_channel[i] < 0) continue;
			float target = md.hip_pos_target[i] + md.hip_sensor_correction_delta[i];

			// (int)-cast is important to prevent overflow in case min and max are reversed here
			u16 target_int = (u16)(cd.servo_min[i] + (int)(target*((int)cd.servo_max[i]-(int)cd.servo_min[i])));
			set_pwm(cd.servo_channel[i], 0, target_int);
		}

		// arm servos
		for (int i = 0; i < 6; i++)
		{
			if (cd.arm_servo_channel[i] < 0) continue;
			float target = md.arm_pos_target_smooth[i];

			u16 servo_min = cd.arm_servo_v2[i] ? cd.arm_servo_min_v2 : cd.arm_servo_min;
			u16 servo_max = cd.arm_servo_v2[i] ? cd.arm_servo_max_v2 : cd.arm_servo_max;

			// (int)-cast is important to prevent overflow in case min and max are reversed here
			u16 target_int = (u16)(servo_min + (int)(target*((int)servo_max-(int)servo_min)));
			set_pwm(cd.arm_servo_channel[i], 0, target_int);
		}
	}

	return true;
}

bool servo_motors_update()
{
	u32_micros start_time = time_micros();
	bool r = servo_motors_update_();
	md.delta_time_servo = time_micros() - start_time;
	return r;
}
