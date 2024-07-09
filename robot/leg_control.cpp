// This handles communication with the ODrive that is connected to the big motors on the legs.
// That is the ODrive that is placed on the front (the side with no batteries) and connected
// to the Jetson via /dev/ttyTHS1 (on the 40-Pin Header).
//
// This ODrive is connected to the leg motors, its sensors (AS5048 Magnetic Rotary Absolute Encoders),
// and also to the CUI AMT Encoders on the hips. This ODrive doesn't really have anything to do with the
// hip, but the Jetson cannot directly communicate with these encoders. So we abuse this ODrive for reading
// the hip encoders because it has free ports for it and it doesn't cost much CPU time. (We need to use
// a custom built firmware for this though).
// 
// Apart from that sensor reading, the code in this file is mainly responsible for these features:
//  - Controlling the leg motors in position control.
//  - Jumping
//  - Side Balancing
//  - Retrieving ODrive oscilloscope data
// 
// Jumping
// The code for jumping has gotten surprisingly complex. A jump is separated into different phases
// (see the enum JumpPhase) and each phase triggers different code. Most of that code is here, but some
// of it is also in the custom ODrive firmware, and also in other modules
// (servo, foot_control, IMU, balance control). The jump itself may not be so complicated, but landing and
// not falling over after a jump turned out to be a quite complex task, and simply requires modifications in
// all those other modules.
//
// Side Balancing
// What's that? That's a name I made up for the mode, where the robot tries to stand straight (no leaning
// to left or right), regardless of what the ground looks like. It's not enabled by default and is used
// for example when the robot drives with one wheel over a ramp. In this mode, the imu side angle is used
// to control the leg motors in such a way, that the side angle is close to zero. Unfortunately, I've never
// gotten this mode to be very stable, that's why it must be enabled explicitly in control_ui.
// The problem is that the actuator of this controller, the changing of the difference of effective leg
// height on each side, doesn't necessarily affect the measured imu side angle directly. It's possible that
// one leg gets airborne, and there is no sensor to detect that. Even if not, it's hard to predict the behavior
// of the whole robot in this scenario. There is also very little leeway. Once the side angle gets so large
// that the center of mass gets over the corresponding wheel, there is nothing the controller can do to
// lean the robot back. Another issue is that in order to handle ramps, to controller needs to be very fast.
// It's implementation is also partly in the ODrive firmware, because the control loop in the robot is
// too slow for it.
//
// Oscilloscope data
// You can also access ODrive oscilloscope data here. That is data that is recorded on ODrive in 8kHz
// and can be used to debug control loop issues. That data can be displayed inline in the plots in control_ui.
// It's very useful to debug issues with both jumping and sidebalancing, especially in combination with
// a video stream.
// 
// Axis numbers are: left = 1, right = 0 (This way the motor wires don't have to cross)

#include "leg_control.h"
#include "../common/odrive/odrive_helper.h"
#include "main.h"
#include "tts_client.h"

#include <string>
#include <math.h>

static void leg_control_update_axis(Endpoint* axis, int leg, float delta_time_unclamped);

ODrive odrive;
static Endpoint* axis0;
static Endpoint* axis1;

static int cd_counter;
static bool motor_running[2];

int oscilloscope_size = 0;

int odrive_enable_side_balance;

bool calibration_leg_done[2];
float calibration_leg_pos[2];
float calibration_leg_rel_pos_max[2];

bool zombie_mode = false;//todo: test this!
float zombie_mode_timer;

inline uint as_uint(const float x) {
    union { float f; uint i; } val;
    val.f = x;
    return val.i;
}
inline float as_float(const uint x) {
    union { float f; uint i; } val;
    val.i = x;
    return val.f;
}

float half_to_float(ushort x) { // IEEE-754 16-bit floating-point format (without infinity): 1-5-10, exp-15, +-131008.0, +-6.1035156E-5, +-5.9604645E-8, 3.311 digits
    const uint e = (x&0x7C00)>>10; // exponent
    const uint m = (x&0x03FF)<<13; // mantissa
    const uint v = as_uint((float)m)>>23; // evil log2 bit hack to count leading zeros in denormalized format
    return as_float((x&0x8000)<<16 | (e!=0)*((e+112)<<23|m) | ((e==0)&(m!=0))*((v-37)<<23|((m<<(150-v))&0x007FE000))); // sign : normalized : denormalized
}

static bool check_errors(Endpoint& root, bool init)
{
	if (odrive.communication_error)
	{
		printf("ODrive Leg communication error\n");
		return false;
	}

	if (md.startup_sequence != SS_LegCalibration1a && md.startup_sequence != SS_LegCalibration1b &&
		(motor_running[0] || motor_running[1]))
	{
		float abs_pos0 = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
		float abs_pos1 =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
		if (abs_pos0 > 1.2f || abs_pos1 > 1.2f ||
			abs_pos0 < -0.75f || abs_pos1 < -0.75f)
		{
			// It is physically impossible to reach these positions
			// so if it happens an encoder is probably defect and we fail here.
			printf("ODrive Leg position out of range: %f %f\n", abs_pos0, abs_pos1);
			return false;
		}
	}

	if (zombie_mode)
	{
		zombie_mode_timer -= md.delta_time;
		if (zombie_mode_timer <= 0)
			return false;
		return true;
	}

	bool any_errors = true;
	root("any_errors_and_watchdog_feed").get(any_errors);
	if (!any_errors)
		return true;

	printf("\nleg odrive error!\n");
	int num_errors = 0;
	int error; root("can")("error").get(error); if (error) { printf("odrive: can error: %d\n", error); num_errors++; }
	check_axis_errors(axis0, "axis0", num_errors);
	check_axis_errors(axis1, "axis1", num_errors);
	
	// DEADLINE_MISSED shouldn't happen, but in practice it sometimes does.
	// We check for that here and then restart the motors.
	// You can actually hear the motors click, but otherwise you don't notice it.
	if (num_errors >= 2 && 
		(*axis0)("error").get2<int>() == AXIS_ERROR_MOTOR_DISARMED &&
		(*axis0)("motor")("error").get2<int>() == 16)
	{
		printf("Clear leg axis0 DEADLINE_MISSED! ######################################\n");
		(*axis0)("error").set(0);
		(*axis0)("motor")("error").set(0);
		num_errors -= 2;
		// Members of ODrive instance on ODrive seem to reset when this happens.
		// So we set those values again now.
		cd_counter = -1;
		odrive_enable_side_balance = -1;
		if (motor_running[0])
			(*axis0)("requested_state").set(AXIS_STATE_CLOSED_LOOP_CONTROL);
	}
	if (num_errors >= 2 && 
		(*axis1)("error").get2<int>() == AXIS_ERROR_MOTOR_DISARMED &&
		(*axis1)("motor")("error").get2<int>() == 16)
	{
		printf("Clear leg axis1 DEADLINE_MISSED! ######################################\n");
		(*axis1)("error").set(0);
		(*axis1)("motor")("error").set(0);
		num_errors -= 2;
		cd_counter = -1;
		odrive_enable_side_balance = -1;
		if (motor_running[1])
			(*axis1)("requested_state").set(AXIS_STATE_CLOSED_LOOP_CONTROL);
	}
	if (num_errors && 
		(((*axis0)("error").get2<int>() & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED) != 0 ||
		((*axis1)("error").get2<int>() & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED) != 0))
	{
		printf("AXIS_ERROR_WATCHDOG_TIMER_EXPIRED\n");
		printf("delta_time_imu: %fms\n", md.delta_time_imu * .001f);
		printf("delta_time_joystick: %fms\n", md.delta_time_joystick * .001f);
		printf("frame counter: %d\n", md.counter);
	}
	if (num_errors && !init && !zombie_mode && (motor_running[0] || motor_running[1]))
	{
		// When we get an ODrive error, we enter "zombie mode".
		// In this mode we keep the main loop running for a while and pretend no error has occured.
		// This has the advantage that we keep getting monitor data and this
		// will then help to debug why this error has occured in the first place.
		// Also, even when both leg motors enter idle mode, there is a chance that
		// the robot can keep balancing and prevent a crash.
		printf("Leg Control: Enter zombie mode.\n");
		zombie_mode = true;
		zombie_mode_timer = 10;
		return true;
	}
	//assert(num_errors != 0);
	return num_errors == 0;
}

void reset_leg_base()
{
	md.leg_base_angle[0] = -1-md.leg_rel_angle[0];
	md.leg_base_angle[1] =  1-md.leg_rel_angle[1];
	(*axis0)("motor")("leg_base_angle").set(md.leg_base_angle[0]);
	(*axis1)("motor")("leg_base_angle").set(md.leg_base_angle[1]);
}

void reset_hip_base()
{
	md.hip_sensor_pos_base[0] = 1-md.hip_sensor_pos_relative[0];
	md.hip_sensor_pos_base[1] = 1-md.hip_sensor_pos_relative[1];
}

float get_side_angle_hip_correction()
{
	float leg_pos_l =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
	float leg_pos_r = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
	float leg_angle_l = (1-leg_pos_l) * (tau / leg_gear_reduction) + leg_angle_1;
	float leg_angle_r = (1-leg_pos_r) * (tau / leg_gear_reduction) + leg_angle_1;
	float hip_angle_l = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[0];
	float hip_angle_r = hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[1];
	float leg_height_upper_l = cos(md.imu_x_angle + hip_angle_l) * upper_leg_length;
	float leg_height_upper_r = cos(md.imu_x_angle + hip_angle_r) * upper_leg_length;
	float leg_height_lower_l = cos(md.imu_x_angle + hip_angle_l + leg_angle_l) * lower_leg_length;
	float leg_height_lower_r = cos(md.imu_x_angle + hip_angle_r + leg_angle_r) * lower_leg_length;
	float leg_height_l = leg_height_upper_l + leg_height_lower_l;
	float leg_height_r = leg_height_upper_r + leg_height_lower_r;

	float side_angle_motor = atan((leg_height_r-leg_height_l)/cd.wheel_side_distance);

	{
		float hip_angle_l = hip_angle_0 + cd.hip_sensor_angle_range*md.servo_pos[0];
		float hip_angle_r = hip_angle_0 + cd.hip_sensor_angle_range*md.servo_pos[1];
		float leg_height_upper_l = cos(md.imu_x_angle + hip_angle_l) * upper_leg_length;
		float leg_height_upper_r = cos(md.imu_x_angle + hip_angle_r) * upper_leg_length;
		float leg_height_lower_l = cos(md.imu_x_angle + hip_angle_l + leg_angle_l) * lower_leg_length;
		float leg_height_lower_r = cos(md.imu_x_angle + hip_angle_r + leg_angle_r) * lower_leg_length;
		float leg_height_l = leg_height_upper_l + leg_height_lower_l;
		float leg_height_r = leg_height_upper_r + leg_height_lower_r;

		float side_angle_motor_target = atan((leg_height_r-leg_height_l)/cd.wheel_side_distance);
		return side_angle_motor_target - side_angle_motor;
	}
}

bool leg_control_init()
{
	if (!odrive.connect_uart("/dev/ttyTHS1", odrive_uart_baudrate, odrive_uart_stop_bits_2)) return false;
	//if (!odrive.connect_usb()) return false;
	if (!odrive.odrive_fw_is_milana)
	{
		printf("Wrong firmware on Leg ODrive\n");
		return false;
	}
	
	axis0 = &odrive.root("axis0");
	axis1 = &odrive.root("axis1");

	odrive.root("vbus_voltage").get(md.battery_total_voltage);

	// Clear potential watchdog errors from previous runs
	(*axis0)("config")("enable_watchdog").set(false);
	(*axis1)("config")("enable_watchdog").set(false);
	if ((*axis0)("error").get2<int>() == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)
	{
		(*axis0)("error").set(0);
		printf("leg: clear watchdog error 0\n");
	}
	if ((*axis1)("error").get2<int>() == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)
	{
		(*axis1)("error").set(0);
		printf("leg: clear watchdog error 1\n");
	}

	motor_running[0] = false;
	motor_running[1] = false;

	odrive_enable_side_balance = 0;
	odrive.root("enable_side_balance").set(false);

	{
		// Retrieve initial sensor values.
		// This ensures that leg_sensor_init sees valid values
		// and we fail early if odrive value cannot be set.

		cd_counter = -1;
		leg_control_update_axis(axis0, 0, 1);
		leg_control_update_axis(axis1, 1, 1);
		cd_counter = cd.odrive_init_counter;
	}
	reset_leg_base();

	if (!check_errors(odrive.root, true))
		return false;
	return true;
}

void leg_control_init2()
{
	// We start the watchdog timer only after we initialized the
	// other odrive. This is because initialization takes so long
	// that this timeout would fire before the first update call.
	(*axis0)("config")("watchdog_timeout").set(0.5f);
	(*axis0)("watchdog_feed").call();
	(*axis0)("config")("enable_watchdog").set(true);

	(*axis1)("config")("watchdog_timeout").set(0.5f);
	(*axis1)("watchdog_feed").call();
	(*axis1)("config")("enable_watchdog").set(true);
}

void leg_control_close()
{
	if (odrive.is_connected)
	{
		(*axis0)("requested_state").set(AXIS_STATE_IDLE);
		(*axis1)("requested_state").set(AXIS_STATE_IDLE);
		(*axis0)("config")("enable_watchdog").set(false);
		(*axis1)("config")("enable_watchdog").set(false);
	}
	odrive.close();
	motor_running[0] = false;
	motor_running[1] = false;
}

static void set_axis_values(Endpoint* axis, int leg)
{
	//printf("set axis %d\n", leg);

	Endpoint& controller_config = (*axis)("controller")("config");
	controller_config("control_mode"          ).set(CONTROL_MODE_POSITION_CONTROL);
	controller_config("vel_gain"              ).set(cd.vel_gain);
	controller_config("pos_gain"              ).set(cd.pos_gain);
	controller_config("vel_integrator_gain"   ).set(cd.vel_integrator_gain);
	(*axis)("trap_traj")("config")("vel_limit").set(cd.vel_limit);
	controller_config("vel_limit"             ).set(cd.vel_limit);
	controller_config("enable_overspeed_error").set(false);
	controller_config("enable_vel_limit"      ).set(false);
	(*axis)("trap_traj")("config")("accel_limit").set(cd.accel_limit);
	(*axis)("trap_traj")("config")("decel_limit").set(cd.accel_limit);

	controller_config("input_filter_bandwidth").set(cd.input_filter_bandwidth);

	(*axis)("encoder")("config")("ignore_abs_ams_error_flag").set(true);
	(*axis)("encoder")("config")("bandwidth").set(cd.encoder_bandwidth);
	(*axis)("encoder")("enable_extra_incremental_counter").set(true);
	(*axis)("encoder")("enable_extra_incremental_counter_index").set(cd.do_hip_sensor_index);
	md.hip_sensor_index_enabled = cd.do_hip_sensor_index;

	(*axis)("motor")("config")("current_lim").set(cd.current_lim);
	(*axis)("motor")("config")("current_lim_margin").set(cd.current_lim_margin);

	(*axis)("motor")("j_vel_gain"           ).set(cd.j_vel_gain);
	(*axis)("motor")("j_pos_gain"           ).set(cd.j_pos_gain);
	(*axis)("motor")("j_vel_integrator_gain").set(cd.j_vel_integrator_gain);
	(*axis)("motor")("j_vel_limit"          ).set(cd.j_vel_limit);
	(*axis)("motor")("j_current_lim_margin" ).set(cd.j_current_lim_margin);
	{
		float current_lim = cd.j_current_lim;
		if (leg == (cd.jump_current_side_deviation < 0))
			current_lim *= 1-abs(cd.jump_current_side_deviation);
		if (current_lim < 0 || current_lim > cd.j_current_lim)
			current_lim = 0;
		(*axis)("motor")("j_current_lim").set(current_lim);
	}

	(*axis)("motor")("f_input_mode"            ).set(cd.f_input_mode);
	(*axis)("motor")("f_vel_gain"              ).set(cd.f_vel_gain);
	(*axis)("motor")("f_pos_gain"              ).set(cd.f_pos_gain);
	(*axis)("motor")("f_vel_integrator_gain"   ).set(cd.f_vel_integrator_gain);
	(*axis)("motor")("f_vel_limit"             ).set(cd.f_vel_limit);
	(*axis)("motor")("f_input_filter_bandwidth").set(cd.f_input_filter_bandwidth);
	(*axis)("motor")("f_current_lim"           ).set(cd.f_current_lim);

	(*axis)("motor")("sl_input_mode"         ).set(cd.input_mode);
	(*axis)("motor")("sl_vel_gain"           ).set(cd.vel_gain);
	(*axis)("motor")("sl_pos_gain"           ).set(cd.pos_gain);
	(*axis)("motor")("sl_vel_integrator_gain").set(cd.vel_integrator_gain);
	(*axis)("motor")("sl_vel_limit"          ).set(cd.vel_limit);
	(*axis)("motor")("sl_initial_integrator" ).set(cd.initial_integrator_after_landing);

	// This is f_current_lim, not current_lim
	// We reset the current limit after postlanding phase
	(*axis)("motor")("sl_current_lim_margin" ).set(cd.j_current_lim_margin);
	(*axis)("motor")("sl_current_lim"        ).set(cd.f_current_lim);
	
	odrive.root("ibus_report_filter_k").set(cd.bus_current_smooth);
	odrive.root("generate_error_on_filtered_ibus").set(cd.generate_error_on_filtered_ibus);

	odrive.root("jump_left_right_timeout").set(cd.jump_left_right_timeout);

	odrive.root("enable_landing_detector").set(false);
	odrive.root("enable_landing_mode").set(false);
	odrive.root("activated_landing").set(false);
	odrive.root("landing_detector_current_threshold").set(cd.landing_detector_current_threshold);
	odrive.root("l_leg_pos_target").set(cd.l_leg_pos_target);
	odrive.root("l_max_time").set(cd.l_max_time);
	odrive.root("l_vel_gain").set(cd.l_vel_gain);
	odrive.root("l_vel_integrator_gain").set(cd.l_vel_integrator_gain);
	odrive.root("l_sync_gain_pos").set(cd.l_sync_gain_pos);
	odrive.root("l_sync_gain_vel").set(cd.l_sync_gain_vel);
	odrive.root("l_initial_measure_time_frames").set(int(cd.l_initial_measure_time*odrive_frequency));
	odrive.root("l_initial_measure_current").set(cd.l_initial_measure_current);
	odrive.root("l_do_max_vel").set(cd.l_do_max_vel);
	odrive.root("l_do_straightening").set(cd.l_do_straightening);
	odrive.root("l_initial_expected_accel_per_frame").set(cd.l_initial_expected_accel_per_s / odrive_frequency);
	odrive.root("l_initial_accel_gain").set(cd.l_initial_accel_gain);
	odrive.root("l_pos_target_delta_per_frame").set(cd.l_pos_target_delta_per_s / odrive_frequency);
	(*axis)("controller")("enable_landing_mode").set(false);
	(*axis)("controller")("l_max_vel").set(0.0f);
	(*axis)("controller")("l_vel_target_delta").set(0.0f);
	(*axis)("controller")("l_vel_target").set(0.0f);
	(*axis)("controller")("l_frame_counter").set(0);

	odrive.root("side_gain_p").set(cd.side_gain_p);
	odrive.root("side_gain_d").set(cd.side_gain_d);
	odrive.root("average_gain_p").set(cd.average_gain_p);
	odrive.root("average_gain_i").set(cd.average_gain_i);
	odrive.root("average_gain_d").set(cd.average_gain_d);
	odrive.root("target_gain_p").set(cd.target_gain_p);
	odrive.root("target_gain_d").set(cd.target_gain_d);
	odrive.root("side_angle_motor_force_limit").set(cd.side_angle_motor_force_limit);
	//odrive.root("stand_control_max_leg_angle").set(cd.stand_control_max_leg_angle);
	//odrive.root("wheel_side_distance").set(cd.wheel_side_distance);
	odrive.root("side_balance_down_limit_factor"   ).set(cd.side_balance_down_limit_factor);
	odrive.root("side_balance_down_limit_threshold").set(cd.side_balance_down_limit_threshold);
	odrive.root("side_balance_max_vertical_angle").set(cd.side_balance_max_vertical_angle);
	odrive.root("side_balance_max_vertical_angle_gain").set(cd.side_balance_max_vertical_angle_gain);
	//odrive.root("acc_smooth_factor").set(cd.acc_smooth_factor);
	odrive.root("acc_current_gain").set(cd.acc_current_gain);
	odrive.root("side_balance_error_d_smooth_factor").set((float)pow(cd.side_balance_error_d_smooth_factor, cd.target_delta_time_ms));

	bool ok = cd.input_mode == INPUT_MODE_PASSTHROUGH ||
		      cd.input_mode == INPUT_MODE_POS_FILTER ||
		      cd.input_mode == INPUT_MODE_TRAP_TRAJ;
	assert(ok);
	if (!ok || md.jump_phase == JP_Landing || md.jump_phase == JP_Jumping || md.jump_phase == JP_LegUp)
		(*axis)("controller")("config")("input_mode").set(INPUT_MODE_PASSTHROUGH);
	else
		(*axis)("controller")("config")("input_mode").set(cd.input_mode);
}

void start_hip_sensor_index()
{
	if (odrive.is_connected)
	{
		(*axis0)("encoder")("enable_extra_incremental_counter_index").set(true);
		(*axis1)("encoder")("enable_extra_incremental_counter_index").set(true);
		md.hip_sensor_index_enabled = true;
	}
}

void stop_hip_sensor_index()
{
	if (odrive.is_connected)
	{
		(*axis0)("encoder")("enable_extra_incremental_counter_index").set(cd.do_hip_sensor_index);
		(*axis1)("encoder")("enable_extra_incremental_counter_index").set(cd.do_hip_sensor_index);
		md.hip_sensor_index_enabled = cd.do_hip_sensor_index;
	}
}

static void leg_control_update_axis(Endpoint* axis, int leg, float delta_time_unclamped)
{
	//(*axis)("watchdog_feed").call(); // called by any_errors_and_watchdog_feed
	if (cd_counter != cd.odrive_init_counter)
		set_axis_values(axis, leg);

	bool should_run = cd.leg_control_enable_motor[leg];
	if (cd.enable_stand_control)
		should_run = true;
	if (md.startup_sequence == SS_Failed)
		should_run = false;
	if (md.startup_sequence >= SS_Hip1 && md.startup_sequence <= SS_ImuCalibration2)
	{
		// Turn motors off during imu calibration.
		// Otherwise they can cause small movements, which screws up the imu calibration.
		should_run = false;
	}
	if (should_run && !motor_running[leg])
	{
		// do not set this again, if it is already set.
		// Otherwise there is a hickup in the motion.
		//(*axis)("controller")("config")("input_mode").set(INPUT_MODE_PASSTHROUGH);
		(*axis)("requested_state").set(AXIS_STATE_CLOSED_LOOP_CONTROL);
		motor_running[leg] = true;
	}
	if (!should_run && motor_running[leg])
	{
		(*axis)("requested_state").set(AXIS_STATE_IDLE);
		motor_running[leg] = false;
	}

	float old_pos = md.leg_rel_angle[leg];
	(*axis)("encoder")("pos_estimate").get(md.leg_rel_angle[leg]);
	//md.leg_pos_error[leg] = md.leg_base_angle[leg]+md.leg_rel_angle[leg] - leg_abs_target[leg];

	md.leg_vel[leg] = (md.leg_rel_angle[leg]-old_pos) / delta_time_unclamped;
	//(*axis)("encoder")("vel_estimate").get(md.leg_vel2[leg]);

	//(*axis)("controller")("vel_setpoint").get(md.leg_vel_target[leg]);

	//(*axis)("motor")("current_control")("Iq_measured")          .get(md.leg_current[leg]);
	if (motor_running[leg])
		(*axis)("motor")("current_control")("Iq_setpoint").get(md.leg_current_target[leg]);
	else
		md.leg_current_target[leg] = 0;
	
	//(*axis)("encoder")("spi_abs_ams_error_flag").get(md.leg_error_flag[leg]);
	//(*axis)("encoder")("spi_error_rate")        .get(md.leg_encoder_error_rate[leg]);
	{
		int no = 1-leg;
		/*
		u16 old_inc = md.hip_sensor_raw1[no];
		(*axis)("encoder")("extra_incremental_counter").get(md.hip_sensor_raw1[no]);
		u16 new_inc = md.hip_sensor_raw1[no];
		if (no == 0)
			md.hip_sensor_raw2[no] += (s16)(new_inc-old_inc);
		else
			md.hip_sensor_raw2[no] -= (s16)(new_inc-old_inc);*/
		
		(*axis)("encoder")("extra_incremental_counter").get(md.hip_sensor_raw1[no]);
		if ((md.startup_sequence >= SS_LegCalibration1b && md.startup_sequence <= SS_Hip2) ||
			md.startup_sequence == SS_Hip3)
		{
			// These values are only checked periodically elsewhere in this file.
			// index_count is used during hip sensor calibration.
			// index_error is checked during SS_Hip3.
			(*axis)("encoder")("index_check_cumulative_error").get(md.hip_sensor_index_error[no]);
			(*axis)("encoder")("index_check_index_count").get(md.hip_sensor_index_count[no]);
		}
		if (no == 0)
			md.hip_sensor_raw2[no] = md.hip_sensor_raw1[no];
		else
			md.hip_sensor_raw2[no] = -md.hip_sensor_raw1[no];

		if (cd.do_hip_sensor_pos_base_manual)
		{
			md.hip_sensor_pos_base[0] = cd.hip_sensor_pos_base_manual[0];
			md.hip_sensor_pos_base[1] = cd.hip_sensor_pos_base_manual[1];
		}


		float old_relative = md.hip_sensor_pos_relative[no];
		md.hip_sensor_pos_relative[no] = (float)md.hip_sensor_raw2[no] * (tau / cd.hip_sensor_angle_range / hip_cpr);
		md.hip_sensor_pos[no] = md.hip_sensor_pos_relative[no] + md.hip_sensor_pos_base[no];

		md.hip_sensor_vel[no] = (md.hip_sensor_pos_relative[no]-old_relative) / md.delta_time;
	}
}

static void set_abs_input_pos0(float abs_target)
{
	md.leg_rel_angle_target[0]                       = -abs_target - md.leg_base_angle[0];
	(*axis0)("controller")("input_pos").set(-abs_target - md.leg_base_angle[0]);
}
static void set_abs_input_pos1(float abs_target)
{
	md.leg_rel_angle_target[1]                       =  abs_target - md.leg_base_angle[1];
	(*axis1)("controller")("input_pos").set( abs_target - md.leg_base_angle[1]);
}

static void handle_side_balance()
{
	// The hip motors have very high backlash, which can cause oscillations.
	// But we have good encoders and calculate the side angle deviation caused by the backlash here
	md.side_angle_hip_correction = get_side_angle_hip_correction();

	float old_side_balance_error = md.side_balance_error;
	md.side_balance_error = md.imu_side_angle;
	// And here we add a force that experimentally counteracts the oscillation.
	md.side_balance_error += md.side_angle_hip_correction * cd.side_angle_hip_correction_factor;
	md.side_balance_error_d = lerp(
			(md.side_balance_error-old_side_balance_error) / md.delta_time,
			md.side_balance_error_d,
			pow(cd.side_balance_error_d_smooth_factor, (float)cd.target_delta_time_ms));

	bool enable_side_balance = false;
	md.common_leg_vel_target = 0;
	if (md.jump_phase == JP_None)
	{
		if (md.startup_sequence == SS_LegCalibration1b || md.startup_sequence == SS_LegCalibration2b)
		{
			md.common_leg_pos_target = md.common_leg_pos_sensor;
			set_abs_input_pos0(calibration_leg_pos[0]);
			set_abs_input_pos1(calibration_leg_pos[1]);
		}
		else if (cd.enable_stand_control)
		{
			if (!cd.enable_side_balance)
			{
				set_abs_input_pos0(md.leg_pos_target_ik[0]);
				set_abs_input_pos1(md.leg_pos_target_ik[1]);
			}
			else
			{
				enable_side_balance = true;

				// When side balance is active and the angle between the upper and lower leg is close to 180°,
				// the sideforce can put one leg to the front and one to the back, and we don't wont that.
				// So we limit the angle here a bit more.
				if (md.common_leg_pos_target < cd.common_leg_pos_min_side_balance)
				{
					md.common_leg_pos_target = move_towards(md.common_leg_pos_target,
							cd.common_leg_pos_min_side_balance,
							cd.common_leg_vel_user_max*md.delta_time);
				}

				if (md.side_balance_recover_mode)
				{
					if (abs(md.imu_side_angle) < cd.side_balance_recover_mode_side_angle_threshold_off)
						md.side_balance_recover_mode_timer += md.delta_time;
					else
						md.side_balance_recover_mode_timer = 0;
					if (md.side_balance_recover_mode_timer >= cd.side_balance_recover_mode_side_angle_time_off)
					{
						md.side_balance_recover_mode = false;
						md.side_balance_recover_mode_timer = 0;
					}
				}
				else
				{
					if (abs(md.imu_side_angle) > cd.side_balance_recover_mode_side_angle_threshold_on)
						md.side_balance_recover_mode_timer += md.delta_time;
					else
					{
						md.side_balance_recover_mode_timer -= md.delta_time;
						if (md.side_balance_recover_mode_timer < 0)
							md.side_balance_recover_mode_timer = 0;
					}
					if (md.side_balance_recover_mode_timer >= cd.side_balance_recover_mode_side_angle_time_on)
					{
						md.side_balance_recover_mode = true;
						md.side_balance_recover_mode_timer = 0;
					}
				}

					
				odrive.root("x_angle_hip_angle_l").set(md.imu_x_angle + hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[0]);
				odrive.root("x_angle_hip_angle_r").set(md.imu_x_angle + hip_angle_0 + cd.hip_sensor_angle_range*md.hip_sensor_pos[1]);
					
				if (cd.side_balance_mode == 0)
				{
					odrive.root("side_angle_motor_target").set(md.imu_side_angle);
					odrive.root("side_balance_mode").set(0);
				}
				else
				{
					if (md.side_balance_recover_mode)
					{
						float target = md.imu_side_angle;
						target = move_towards(target, 0, cd.side_balance_recover_mode_side_angle_delta);
						odrive.root("side_angle_motor_target").set(target);
						odrive.root("side_balance_mode").set(0);
					}
					else
					{
						odrive.root("side_balance_mode").set(1);
						odrive.root("side_angle").set(md.side_balance_error);
						md.side_angle_target = cd.side_angle_target;
						odrive.root("side_angle_target").set(md.side_angle_target);
					}
				}
				odrive.root("average_leg_pos_target").set(md.common_leg_pos_target);
				odrive.root("average_leg_pos_integrator").get(md.average_leg_pos_integrator);
			}
		}
		else
		{
			md.common_leg_pos_target = md.common_leg_pos_sensor;
			if (cd.leg_control_enable_motor[0]) set_abs_input_pos0(cd.leg_abs_target[0]);
			if (cd.leg_control_enable_motor[1]) set_abs_input_pos1(cd.leg_abs_target[1]);
		}
	}
	if (odrive_enable_side_balance != (int)enable_side_balance)
	{
		odrive_enable_side_balance = (int)enable_side_balance;
		odrive.root("enable_side_balance").set(enable_side_balance);
	}
}

static void handle_jumping_1()
{
	// handle jump triggers
	// This needs to happen before the oscilloscope handling stuff to ensure
	// that the oscilloscope records the entire beginning of the jump.

	// jump trigger
	static int last_jump_phase_trigger;
	if (cd.jump_phase_trigger-last_jump_phase_trigger == 1 &&
		md.jump_phase == JP_None && md.is_standing)
	{
		if (md.common_leg_pos_target > 0.90f)
		{
			if (!cd.enable_side_balance)
			{
				md.jump_phase = (cd.do_positioning ? JP_Positioning : JP_Jumping);
			}
			else
				printf("Cannot jump with side balance activated!\n");
		}
		else
		{
			printf("Leg angle must be lower to jump!\n");
			tts_say_text("Leg angle must be lower to jump.");
		}
	}
	last_jump_phase_trigger = cd.jump_phase_trigger;

	// fall trigger
	// This is only used for debugging the fall phase.
	static int last_fall_phase_trigger;
	if (cd.fall_phase_trigger-last_fall_phase_trigger == 1 &&
		(md.jump_phase == JP_None || md.jump_phase == JP_Jumping || md.jump_phase == JP_Falling))
	{
		if (md.jump_phase == JP_Falling)
		{
			md.force_stay_in_falling_mode = false;
		}
		else
		{
			md.jump_phase = JP_Falling;
			md.force_stay_in_falling_mode = true;

			float hip_angle = hip_angle_0 + cd.hip_sensor_angle_range*md.common_hip_pos_sensor;
			md.l_base_angle =
					cd.l_base_angle_delta +
					md.imu_x_angle +
					hip_angle;
			odrive.root("l_base_angle").set(md.l_base_angle);
		}
	}
	last_fall_phase_trigger = cd.fall_phase_trigger;
	if (md.jump_phase != JP_Falling)
	{
		// This should already be false, but better safe than sorry!
		md.force_stay_in_falling_mode = false;
	}

	if (md.jump_phase == JP_Positioning)
	{
		// During positioning, we lean the robot a bit forward and as soon as we
		// reach a certain threshold, we jump.
		// This usually takes about 100ms at most.
		// If we didn't do this and error_p is a bit low, this angle can get very low during the jump
		// and eventually cause the robot to be unable to land properly.
		// monitor_log_jump82.bin has some examples of this.
		if ((md.error_p-md.avg_error_p) > cd.positioning_error_p_threshold)
		{
			md.jump_phase = JP_Jumping;
		}
	}
}

static void handle_jumping_2()
{
	// Handle jump phases

	bool start_leg_up_phase = false;
	if (md.jump_phase == JP_Jumping)
	{
		if (md.jump_timer == 0)
		{
			(*axis0)("motor")("jump_pos_target").set(-cd.jump_pos_target - md.leg_base_angle[0]);
			(*axis1)("motor")("jump_pos_target").set( cd.jump_pos_target - md.leg_base_angle[1]);
			md.leg_rel_angle_target[0]             = -cd.jump_pos_target - md.leg_base_angle[0];
			md.leg_rel_angle_target[1]             =  cd.jump_pos_target - md.leg_base_angle[1];

			float hip_angle = hip_angle_0 + cd.hip_sensor_angle_range*md.common_hip_pos_sensor;
			md.l_base_angle =
					cd.l_base_angle_delta +
					md.imu_x_angle +
					hip_angle;
			odrive.root("l_base_angle").set(md.l_base_angle);

			//printf("trigger jump\n");
			// This does the same as the commented code below, but faster.
			// Also, this command ensures both legs start jumping at the same
			// odrive frame.
			odrive.root("trigger_jump").set(true);

			/*for (Endpoint* axis : axes)
			{
				Endpoint& controller_config = (*axis)("controller")("config");
				controller_config("input_mode").set(INPUT_MODE_PASSTHROUGH);
				controller_config("vel_gain"              ).set(cd.j_vel_gain);
				controller_config("pos_gain"              ).set(cd.j_pos_gain);
				controller_config("vel_integrator_gain"   ).set(cd.j_vel_integrator_gain);
				(*axis)("trap_traj")("config")("vel_limit").set(cd.j_vel_limit);
				controller_config("vel_limit"             ).set(cd.j_vel_limit*5.0f);
				(*axis)("motor")("config")("current_lim_margin").set(cd.j_current_lim_margin);
			}
			(*axis0)("motor")("config")("current_lim").set(cd.j_current_lim);
			(*axis1)("motor")("config")("current_lim").set(cd.j_current_lim);
			(*axis0)("controller")("input_pos").set(-cd.jump_pos_target - md.leg_base_angle[0]);
			(*axis1)("controller")("input_pos").set( cd.jump_pos_target - md.leg_base_angle[1]);*/
		}

		// Did we reach the target? Start falling
		if (md.common_leg_pos_sensor < cd.jump_pos_target+0.3f)
		{
			md.jump_phase = cd.do_jump_leg_up ? JP_LegUp : JP_Falling;
			start_leg_up_phase = true;
		}

		// The jump phase usually lasts less than 0.1 seconds
		// This is a safety measure, because the jump phase can
		// be quite dangerous if it lasts too long.
		if (md.jump_timer > 0.2)
		{
			printf("jump phase cancelled!\n");
			md.jump_phase = JP_Falling;
		}
	}

	if (md.jump_phase == JP_LegUp)
	{
		if (start_leg_up_phase)
		{
			(*axis0)("motor")("trigger_falling").set(true);
			(*axis1)("motor")("trigger_falling").set(true);
			//(*axis0)("controller")("config")("input_filter_bandwidth").set(cd.leg_up_input_filter_bandwidth);
			//(*axis1)("controller")("config")("input_filter_bandwidth").set(cd.leg_up_input_filter_bandwidth);
			set_abs_input_pos0(cd.leg_up_pos_target);
			set_abs_input_pos1(cd.leg_up_pos_target);
		}

		if (md.jump_timer > cd.leg_up_min_time)
			md.jump_phase = JP_Falling;

		if (md.jump_timer > 0.3)
		{
			printf("leg up phase cancelled!\n");
			md.jump_phase = JP_Falling;
		}
	}
		
	static float fall_phase_timer;
	if (md.jump_phase == JP_Falling)
	{
		if (fall_phase_timer == 0)
		{
			(*axis0)("motor")("trigger_falling").set(true);
			(*axis1)("motor")("trigger_falling").set(true);

			/*std::array<Endpoint*, 2> axes = {axis0, axis1};
			for (Endpoint* axis : axes)
			{
				Endpoint& controller_config = (*axis)("controller")("config");
				controller_config("input_mode").set(cd.input_mode);
				controller_config("vel_gain"              ).set(cd.vel_gain);
				controller_config("pos_gain"              ).set(cd.pos_gain);
				controller_config("vel_integrator_gain"   ).set(cd.vel_integrator_gain);
				(*axis)("trap_traj")("config")("vel_limit").set(cd.vel_limit);
				controller_config("vel_limit"             ).set(cd.vel_limit*5.0f);
				controller_config("input_filter_bandwidth").set(cd.f_input_filter_bandwidth);
				(*axis)("motor")("config")("current_lim"  ).set(cd.f_current_lim);
				// We do not set current_lim_margin back to normal here yet.
				// We do this at the end of landing.
			}*/
		}
		fall_phase_timer += md.delta_time;

		// Convert angle from imu to target leg position,
		// so that the lower leg is a specified angle to the ground.
		// This also depends on the hip angle, because the imu is in
		// the body and we need to calculate the angle of the upper leg.
			
		{
			float hip_angle = lerp(hip_angle_0, hip_angle_0+cd.hip_sensor_angle_range, md.hip_pos_target[0]);

			float x = cd.fall_vertical_angle - (md.imu_x_angle + hip_angle + leg_angle_1);
			float abs_target = 1 - (x * leg_gear_reduction / tau);
			abs_target = clamp(abs_target, cd.fall_min_leg_angle, cd.fall_max_leg_angle);
		
			set_abs_input_pos1(abs_target);
		}
		{
			float hip_angle = lerp(hip_angle_0, hip_angle_0+cd.hip_sensor_angle_range, md.hip_pos_target[1]);

			float x = cd.fall_vertical_angle - (md.imu_x_angle + hip_angle + leg_angle_1);
			float abs_target = 1 - (x * leg_gear_reduction / tau);
			abs_target = clamp(abs_target, cd.fall_min_leg_angle, cd.fall_max_leg_angle);
		
			set_abs_input_pos0(abs_target);
		}
			
		
		if (!md.enabled_landing_detector &&
			md.jump_timer > cd.landing_detector_delay &&
			!md.force_stay_in_falling_mode)
		{
			// Enable landing_detector on ODrive
			// This is a mode added to the ODrive firmware by this project
			// that detects touchdown with the ground and then changes
			// the pid gains for a soft landing.
			// This is a bit delayed because the current is a bit jumpy right
			// after the jump.
			md.enabled_landing_detector = true;
			(*axis0)("controller")("config")("input_filter_bandwidth").set(cd.input_filter_bandwidth);
			(*axis1)("controller")("config")("input_filter_bandwidth").set(cd.input_filter_bandwidth);
			odrive.root("enable_landing_detector").set(true);
		}
	}
	else
		fall_phase_timer = 0;
		
	if (md.enabled_landing_detector && odrive.root("activated_landing").get2<bool>())
	{
		// ODrive just detected touchdown with the ground and
		// initiated landing mode.
		// This needs to happen there, because our control loop is way too slow
		// to react fast enough.
		md.enabled_landing_detector = false;
		md.jump_phase = JP_Landing;
		md.post_landing_timer = 0;

		md.leg_rel_angle_target[0] = -cd.l_leg_pos_target - md.leg_base_angle[0];
		md.leg_rel_angle_target[1] =  cd.l_leg_pos_target - md.leg_base_angle[1];

		// During the jump, the wheels are turned off and rotate more or less randomly.
		// This may build up a rotation error, that doesn't really exist, so we reset it.
		md.balance_control_rotation_error_bias = -md.foot_pos[0] - md.foot_pos[1];
	}
	md.post_landing_timer += md.delta_time;

	if (md.jump_phase == JP_Landing)
	{
		if (!odrive.root("enable_landing_mode").get2<bool>())
		{
			//printf("stop landing!\n");
			odrive.root("activated_landing").set(false);

			md.common_leg_pos_target = cd.l_leg_pos_target;
			md.leg_rel_angle_target[0] = -cd.l_leg_pos_target - md.leg_base_angle[0];
			md.leg_rel_angle_target[1] =  cd.l_leg_pos_target - md.leg_base_angle[1];

			md.jump_phase = JP_PostLanding;
		}
	}

	static float post_landing_phase_timer;
	if (md.jump_phase == JP_PostLanding)
	{
		post_landing_phase_timer += md.delta_time;
		if (post_landing_phase_timer > cd.post_landing_time)
		{
			(*axis0)("motor")("config")("current_lim").set(cd.current_lim);
			(*axis0)("motor")("config")("current_lim_margin").set(cd.current_lim_margin);
			(*axis1)("motor")("config")("current_lim").set(cd.current_lim);
			(*axis1)("motor")("config")("current_lim_margin").set(cd.current_lim_margin);

			md.jump_phase = JP_None;
		}
	}
	else
		post_landing_phase_timer = 0;

	if (md.jump_phase >= JP_Jumping)
		md.jump_timer += md.delta_time;
	else
		md.jump_timer = 0;
}

static void handle_oscilloscope()
{
	bool trigger_oscilloscope = false;
		
	static int last_oscilloscope_force_trigger;
	if (cd.oscilloscope_force_trigger == last_oscilloscope_force_trigger+1)
		trigger_oscilloscope = true;
	last_oscilloscope_force_trigger = cd.oscilloscope_force_trigger;
	
	if (cd.start_oscilloscope_on_jump && (md.jump_phase == JP_Jumping || md.jump_phase == JP_Landing))
	//if (cd.start_oscilloscope_on_jump && (md.jump_phase == JP_Falling || md.jump_phase == JP_Landing))
		trigger_oscilloscope = true;

	if (cd.start_oscilloscope_on_ramp && cd.enable_side_balance && md.is_standing &&
		abs(md.leg_rel_angle[0]+md.leg_base_angle[0]+md.leg_rel_angle[1]+md.leg_base_angle[1]) > 0.35f)
	{
		trigger_oscilloscope = true;
		md.post_ramp_timer = 0;
	}
	md.post_ramp_timer += md.delta_time;

	if (trigger_oscilloscope && md.oscilloscope_state == 0)
	{
		(*axis0)("motor")("oscilloscope_force_trigger").set(true);
		s64 oscilloscope_counter = 0;
		while (oscilloscope_counter == 0 && running)
		{
			(*axis0)("motor")("oscilloscope_counter").get(oscilloscope_counter);
		}

		if ((oscilloscope_counter & ((s64)1<<62)) == 0)
		{
			printf("oscilloscope doesn't seem to work!\n");
			(*axis0)("motor")("oscilloscope_force_trigger").set(false);
		}
		else
		{
			md.odrive_counter = (int)oscilloscope_counter;
			md.oscilloscope_start = md.odrive_counter;
			md.oscilloscope_end = md.odrive_counter;
			md.oscilloscope_state = 1;
		}
	}
	else if (md.oscilloscope_state == 1)
	{
		s64 oscilloscope_counter = (*axis0)("motor")("oscilloscope_counter").get2<s64>();
		if ((oscilloscope_counter & ((s64)1<<61)) != 0)
		{
			// oscilloscope recording is done
			md.odrive_counter = (int)oscilloscope_counter;
			md.oscilloscope_end = md.odrive_counter;
			md.oscilloscope_state = 2;
			(*axis0)("motor")("oscilloscope_counter").set((s64)0);
		}
	}
	else if (md.oscilloscope_state == 2)
	{
		odrive.root("oscilloscope_size").get(oscilloscope_size);
		md.oscilloscope_state = 3;
		md.oscilloscope_start = 0;
		md.oscilloscope_end = 0;
	}
	if (md.oscilloscope_state == 3 && md.jump_phase == JP_None &&
		md.post_landing_timer > 5 && md.post_ramp_timer > 4)
	{
		// Transmit recorded values
		md.oscilloscope_start = md.oscilloscope_end;
		while (md.oscilloscope_end < oscilloscope_size &&
			    md.oscilloscope_end-md.oscilloscope_start < oscilloscope_transmitting_size)
		{
#if 0
			float value = -1;
			odrive.root("get_oscilloscope_val").call(md.oscilloscope_end, &value);
			md.oscilloscope_transmitting[md.oscilloscope_end-md.oscilloscope_start] = value;
			md.oscilloscope_end++;
#else
			// speed up oscilloscope retrieval by sending 4 values, packed into 64bit, at once.
			u64 value = 0;
			odrive.root("get_oscilloscope_val_4").call(md.oscilloscope_end, &value);
			int i = 0;
			while (md.oscilloscope_end < oscilloscope_size &&
					md.oscilloscope_end-md.oscilloscope_start < oscilloscope_transmitting_size &&
					i < 4)
			{
				md.oscilloscope_transmitting[md.oscilloscope_end-md.oscilloscope_start] = half_to_float((u16)(value>>(i*16)));
				md.oscilloscope_end++;
				i++;
			}
#endif
		}
		if (md.oscilloscope_start == md.oscilloscope_end)
			md.oscilloscope_state = 0;
	}
}

bool leg_control_update()
{
	u32_micros start_time = time_micros();

	static u32_micros last_time = 0;
	float delta_time_unclamped = float(start_time-last_time) * .000001f;
	last_time = start_time;

	static int last_reset_leg_sensors;
	if (cd.reset_leg_sensors_trigger-last_reset_leg_sensors == 1)
	{
		reset_leg_base();
	}
	last_reset_leg_sensors = cd.reset_leg_sensors_trigger;

	static int last_reset_hip_sensors;
	if (cd.reset_hip_sensors_trigger-last_reset_hip_sensors == 1)
	{
		reset_hip_base();
	}
	last_reset_hip_sensors = cd.reset_hip_sensors_trigger;

	md.startup_sequence_timer += md.delta_time;
	static int last_startup_sequence;
	if (cd.startup_sequence_trigger-last_startup_sequence == 1)
	{
		if (md.startup_sequence >= SS_LegCalibration1a)
			printf("startup sequence already in progress!\n");
		else if (md.is_standing)
			printf("Cannot startup while standing!\n");
		else
		{
			//printf("start startup sequence\n");
			md.startup_sequence = SS_LegCalibration1a;
		}
	}
	last_startup_sequence = cd.startup_sequence_trigger;

	if (md.startup_sequence == SS_LegCalibration1a || md.startup_sequence == SS_LegCalibration2a)
	{
		// We do leg calibration twice during the startup sequence.
		// This is because the first calibration might get distorted due to
		// wrong initial hip motor rotations.
		calibration_leg_done[0] = false;
		calibration_leg_done[1] = false;
		calibration_leg_pos[0] = -(md.leg_rel_angle[0]+md.leg_base_angle[0]);
		calibration_leg_pos[1] =  (md.leg_rel_angle[1]+md.leg_base_angle[1]);
		calibration_leg_rel_pos_max[0] = -md.leg_rel_angle[0];
		calibration_leg_rel_pos_max[1] =  md.leg_rel_angle[1];
		if (md.startup_sequence == SS_LegCalibration1a)
		{
			// We force hip sensor index on in the very beginning (SS_LegCalibration1a)
			// and end it once it is confirmed it was found
			// and it doesn't trigger at different positions (end of SS_Hip3).
			// We end it because in case it triggers due to EM noise, we don't want the false index
			// to mess up our data. (The z-index seems to be much more suspectible to noise than
			// the AB lines)
			start_hip_sensor_index();
			md.startup_sequence = SS_LegCalibration1b;
		}
		else
			md.startup_sequence = SS_LegCalibration2b;
	}
	if (md.startup_sequence == SS_LegCalibration1b || md.startup_sequence == SS_LegCalibration2b)
	{
		if (!calibration_leg_done[0])
		{
			calibration_leg_pos[0] += cd.startup_leg_vel*md.delta_time;
			if (-md.leg_current_target[0] > cd.startup_leg_current_threshold)
				calibration_leg_done[0] = true;
		}
		if (!calibration_leg_done[1])
		{
			calibration_leg_pos[1] += cd.startup_leg_vel*md.delta_time;
			if (md.leg_current_target[1] > cd.startup_leg_current_threshold)
				calibration_leg_done[1] = true;
		}
		calibration_leg_rel_pos_max[0] = std::max(calibration_leg_rel_pos_max[0], -md.leg_rel_angle[0]);
		calibration_leg_rel_pos_max[1] = std::max(calibration_leg_rel_pos_max[1],  md.leg_rel_angle[1]);
		if (calibration_leg_done[0] && calibration_leg_done[1])
		{
			md.leg_base_angle[0] = -(1-calibration_leg_rel_pos_max[0]);
			md.leg_base_angle[1] =  (1-calibration_leg_rel_pos_max[1]);
			(*axis0)("motor")("leg_base_angle").set(md.leg_base_angle[0]);
			(*axis1)("motor")("leg_base_angle").set(md.leg_base_angle[1]);
			if (md.startup_sequence == SS_LegCalibration1b)
				md.startup_sequence = SS_Hip1;
			else
				md.startup_sequence = SS_Hip3;
			md.common_hip_pos_target = 1;
		}
	}

	if (cd.enable_leg_control)
	{
		// handle oscilloscope
		int new_counter = (*axis0)("loop_counter").get2<int>();
		if (md.odrive_counter == new_counter)
			printf("odrive_counter did not change! %d\n", new_counter);
		if (md.odrive_counter > new_counter)
			printf("odrive_counter overflow! %d %d\n", md.odrive_counter, new_counter);
		md.odrive_counter = new_counter;

		handle_jumping_1();

		handle_oscilloscope();

		leg_control_update_axis(axis0, 0, delta_time_unclamped);
		leg_control_update_axis(axis1, 1, delta_time_unclamped);

		if (md.counter%512 == 0)
		{
			float temperature0 = (*axis0)("fet_thermistor")("temperature").get2<float>();
			float temperature1 = (*axis1)("fet_thermistor")("temperature").get2<float>();
			md.odrive_temperature = std::max(temperature0, temperature1);
		}

		//printf("ODrive temp: %.2fC\n", md.odrive_temperature);
		cd_counter = cd.odrive_init_counter;

		md.common_leg_pos_sensor = (-(md.leg_rel_angle[0]+md.leg_base_angle[0])+(md.leg_rel_angle[1]+md.leg_base_angle[1]))*0.5f;
		md.common_hip_pos_sensor = (md.hip_sensor_pos[0] + md.hip_sensor_pos[1]) * 0.5f;

		handle_side_balance();

		handle_jumping_2();

		//(*axis0)("controller")("pos_setpoint").get(md.leg_rel_angle_target [0]);
		//(*axis1)("controller")("pos_setpoint").get(md.leg_rel_angle_target [1]);
		//odrive.root("debug1").get(md.leg_control_debug1);
		//odrive.root("debug2").get(md.leg_control_debug2);
		//odrive.root("debug3").get(md.leg_control_debug3);

		// We want these variables for debugging purposes, but don't want to waste too much time
		// retrieving them. So we retrieve only one variable per timestep.
		switch (md.counter%17)
		{
		case 0: (*axis0)("controller")("input_pos").get(md.leg_rel_angle_target[0]);     break;
		case 1: (*axis1)("controller")("input_pos").get(md.leg_rel_angle_target[1]);     break;

		case 2: (*axis0)("controller")("config")("pos_gain").get(md.leg_control_pos_gain_check[0]);	 break;
		case 3: (*axis1)("controller")("config")("pos_gain").get(md.leg_control_pos_gain_check[1]);	 break;
		
		case 4: odrive.root("vbus_voltage").get(md.battery_total_voltage);               break;

		case 5: (*axis0)("encoder")("index_check_cumulative_error").get(md.hip_sensor_index_error[1]); break;
		case 6: (*axis1)("encoder")("index_check_cumulative_error").get(md.hip_sensor_index_error[0]); break;
		case 7: (*axis0)("encoder")("index_check_index_count"     ).get(md.hip_sensor_index_count[1]); break;
		case 8: (*axis1)("encoder")("index_check_index_count"     ).get(md.hip_sensor_index_count[0]); break;

		case 9 :(*axis0)("controller")("vel_integrator_torque").get(md.leg_integrator[0]); break;
		case 10:(*axis1)("controller")("vel_integrator_torque").get(md.leg_integrator[1]); break;

		case 11: odrive.root("ibus").get(md.leg_control_bus_current); break;
		// 12-16 are handled in foot_control_update_2!
		}
	}
	else
	{
		odrive.root("vbus_voltage").get(md.battery_total_voltage);
		if (motor_running[0])
		{
			(*axis0)("requested_state").set(AXIS_STATE_IDLE);
			motor_running[0] = false;
		}
		if (motor_running[1])
		{
			(*axis1)("requested_state").set(AXIS_STATE_IDLE);
			motor_running[1] = false;
		}
		md.leg_vel[0] = 0;
		md.leg_vel[1] = 0;
		md.leg_current[0] = 0;
		md.leg_current[1] = 0;
		md.common_leg_pos_sensor = 1;
	}
	if (!check_errors(odrive.root, false))
		return false;
	
	md.leg_control_endpoint_request_count = odrive.endpoint_request_counter;
	odrive.endpoint_request_counter = 0;

	md.delta_time_leg_control = time_micros() - start_time;

	return true;
}
