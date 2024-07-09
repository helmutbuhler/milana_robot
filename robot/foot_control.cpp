// This handles communication with the ODrive that is connected to the motors on the wheels (the feet).
// That is the ODrive that is placed on the back (the side with the batteries) and connected
// to the Jetson via /dev/ttyS0 (the debug serial port).
//
// Here we basically take the motor speed that is calculated in balance_control.cpp and forward
// that speed to the ODrive.
// We also retrieve the encoder values here.
// 
// Because balance_control needs our encoder values and outputs the target speed, there is a circular
// dependency there. We break that up by splitting our update function in two parts.
// First we retrieve the sensor values in part one, then balance control does its calculation
// and then we set the motor target speed in part two.
//
// We mostly use ODrive in velocity control. This way ODrive can handle the conversion into current
// in a much faster running control loop. Only during some phases of jumping we switch to current control
// because it worked out better that way in experiments. The robot needs to keep its side angle close
// to 0 during jumping, because it has no way to correct that when it is in the air. Current control seems
// to help with that during the jumping phase.
//
// Axis numbers are: left = 0, right = 1 (This way the motor wires don't have to cross)


#include "foot_control.h"
#include "../common/odrive/odrive_helper.h"
#include "main.h"
#include "command.h"

#include <string>

float foot_control_update_axis(Endpoint& axis, int axis_no);

ODrive odrv_foot;
static int cd_counter;
static bool motor_running[2];
static bool is_in_torque_mode = false;

static bool check_errors(Endpoint& root)
{
	if (odrv_foot.communication_error)
	{
		printf("ODrive Foot communication error\n");
		return false;
	}
	bool any_errors = true;
	root("any_errors_and_watchdog_feed").get(any_errors);
	if (!any_errors)
		return true;

	printf("\nfoot odrive error!\n");
	int num_errors = 0;
	int error; root("can")("error").get(error); if (error) { printf("odrv_foot: can error: %d\n", error); num_errors++; }
	check_axis_errors(&root("axis0"), "axis0", num_errors);
	check_axis_errors(&root("axis1"), "axis1", num_errors);
	
	if (num_errors >= 2 && 
		root("axis0")("error").get2<int>() == AXIS_ERROR_MOTOR_DISARMED &&
		root("axis0")("motor")("error").get2<int>() == 16)
	{
		printf("Clear foot axis0 DEADLINE_MISSED! ######################################\n");
		root("axis0")("error").set(0);
		root("axis0")("motor")("error").set(0);
		num_errors -= 2;
		if (motor_running[0])
			root("axis0")("requested_state").set(AXIS_STATE_CLOSED_LOOP_CONTROL);
	}
	if (num_errors >= 2 && 
		root("axis1")("error").get2<int>() == AXIS_ERROR_MOTOR_DISARMED &&
		root("axis1")("motor")("error").get2<int>() == 16)
	{
		printf("Clear foot axis1 DEADLINE_MISSED! ######################################\n");
		root("axis1")("error").set(0);
		root("axis1")("motor")("error").set(0);
		num_errors -= 2;
		if (motor_running[1])
			root("axis1")("requested_state").set(AXIS_STATE_CLOSED_LOOP_CONTROL);
	}
	if (num_errors && 
		((root("axis0")("error").get2<int>() & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED) != 0 ||
		(root("axis1")("error").get2<int>() & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED) != 0))
	{
		printf("AXIS_ERROR_WATCHDOG_TIMER_EXPIRED\n");
		printf("delta_time_imu: %fms\n", md.delta_time_imu * .001f);
		printf("delta_time_joystick: %fms\n", md.delta_time_joystick * .001f);
		printf("delta_time_network: %fms\n", md.delta_time_network * .001f);
		printf("delta_time_leg_control: %fms\n", md.delta_time_leg_control * .001f);
		printf("delta_time_battery: %fms\n", md.delta_time_battery * .001f);
		printf("delta_time_foot_control_1: %fms\n", md.delta_time_foot_control_1 * .001f);
		printf("delta_time_foot_control_2: %fms\n", md.delta_time_foot_control_2 * .001f);
		printf("frame counter: %d\n", md.counter);
	}
	//assert(num_errors != 0);
	return num_errors == 0;
}

bool foot_control_init()
{
	if (!odrv_foot.connect_uart("/dev/ttyS0", odrive_uart_baudrate, odrive_uart_stop_bits_2)) return false;
	//if (!odrv_foot.connect_usb()) return false;
	if (!odrv_foot.odrive_fw_is_milana)
	{
		printf("Wrong firmware on Foot ODrive\n");
		return false;
	}

	// Clear potential watchdog errors from previous runs
	odrv_foot.root("axis0")("config")("enable_watchdog").set(false);
	odrv_foot.root("axis1")("config")("enable_watchdog").set(false);
	if (odrv_foot.root("axis0")("error").get2<int>() == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)
	{
		odrv_foot.root("axis0")("error").set(0);
		printf("clear watchdog error 0\n");
	}
	if (odrv_foot.root("axis1")("error").get2<int>() == AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)
	{
		odrv_foot.root("axis1")("error").set(0);
		printf("clear watchdog error 1\n");
	}

	motor_running[0] = false;
	motor_running[1] = false;

	{
		// Retrieve initial sensor values so we fail early if foot_init_axis fails for some reason.

		cd_counter = -1;
		foot_control_update_axis(odrv_foot.root("axis0"), 0);
		foot_control_update_axis(odrv_foot.root("axis1"), 1);
		cd_counter = cd.odrive_init_counter;
	}

	if (!check_errors(odrv_foot.root))
		return false;
	return true;
}

void foot_control_close()
{
	if (odrv_foot.is_connected)
	{
		odrv_foot.root("axis0")("requested_state").set(AXIS_STATE_IDLE);
		odrv_foot.root("axis1")("requested_state").set(AXIS_STATE_IDLE);
		odrv_foot.root("axis0")("config")("enable_watchdog").set(false);
		odrv_foot.root("axis1")("config")("enable_watchdog").set(false);
	}
	odrv_foot.close();
	motor_running[0] = false;
	motor_running[1] = false;
}

void foot_init_axis(Endpoint& axis, int axis_no)
{
	axis("motor")("config")("current_lim_margin").set(cd.foot_control_current_lim_margin);

	Endpoint& controller_config = axis("controller")("config");
	controller_config("vel_gain"              ).set(cd.foot_control_vel_gain);
	controller_config("vel_integrator_gain"   ).set(cd.foot_control_vel_integrator_gain);
	controller_config("vel_limit"             ).set(cd.foot_control_vel_limit);
	controller_config("vel_limit_tolerance"   ).set(1.5f);
	controller_config("enable_vel_limit"      ).set(true);
	controller_config("enable_overspeed_error").set(true);
	controller_config("anticogging")("anticogging_enabled").set(cd.foot_control_enable_anticogging);
	controller_config("control_mode").set(CONTROL_MODE_VELOCITY_CONTROL);
	is_in_torque_mode = false;
	controller_config("input_mode").set(INPUT_MODE_PASSTHROUGH);
	axis("controller")("input_torque").set(cd.foot_control_torque_target[axis_no]);
	
	odrv_foot.root("ibus_report_filter_k").set(cd.bus_current_smooth);
	odrv_foot.root("generate_error_on_filtered_ibus").set(cd.generate_error_on_filtered_ibus);

	axis("config")("watchdog_timeout").set(0.5f);
	axis("watchdog_feed").call();
	axis("config")("enable_watchdog").set(true);
}

float foot_control_update_axis(Endpoint& axis, int axis_no)
{
	//axis("watchdog_feed").call(); // called by any_errors_and_watchdog_feed
	if (cd_counter != cd.odrive_init_counter)
		foot_init_axis(axis, axis_no);

	float wanted_current_lim = cd.foot_control_current_lim;
	if (md.jump_phase == JP_Landing || md.post_landing_timer < 1)
		wanted_current_lim = cd.foot_control_current_lim_landing;
	if (wanted_current_lim != md.foot_set_current_lim[axis_no])
	{
		axis("motor")("config")("current_lim").set(wanted_current_lim);
		md.foot_set_current_lim[axis_no] = wanted_current_lim;
	}


	bool should_run = cd.foot_control_enable_motor[axis_no];
	if (cd.balance_control_enable && md.is_standing)
		should_run = true;
	if (md.startup_sequence == SS_FootIndex2 && md.foot_motor_ready[axis_no])
		should_run = true;

	if (should_run && !motor_running[axis_no])
	{
		axis("requested_state").set(AXIS_STATE_CLOSED_LOOP_CONTROL);
		motor_running[axis_no] = true;
	}
	if (!should_run && motor_running[axis_no])
	{
		axis("requested_state").set(AXIS_STATE_IDLE);
		motor_running[axis_no] = false;
	}

	float input_vel = 0;
	if (motor_running[axis_no])
		input_vel = cd.foot_control_vel_target[axis_no];
	if (cd.balance_control_enable)
	{
		// Basic velocity based on user input and balance control
		input_vel = md.balance_control_vel_output;

		// rotation velocity, different sign on each side
		input_vel += md.balance_control_rotation_vel * (axis_no == 1 ? 1 : -1);

		// apply velocity error correction. See balance control where this value is calculated.
		input_vel += axis_no == 0 ? md.balance_control_delta_vel_l : md.balance_control_delta_vel_r;

		// There is also some rotation error correction, to compensate for unwanted rotations.
		input_vel += md.balance_control_rotation_error * cd.balance_control_rotation_error_factor * (axis_no == 1 ? 1 : -1);

		// motors are mounted symmetrical, need to negate one side.
		if (axis_no == 1)
			input_vel = -input_vel;
	}
	if (md.startup_sequence == SS_FootIndex2 && md.foot_motor_ready[axis_no])
		input_vel = 0.5f * (axis_no == 1 ? 1 : -1);
	
	if (!should_run)
		input_vel = 0;

	md.foot_control_limit_accel = true;
	if (abs((axis_no == 0 ? 1 : -1)*md.foot_vel_target[axis_no]-md.balance_control_vel_quasi_smooth) > 4)
	{
		// The current speed is much faster than intended.
		// This can happen when the wheels lose contact with the ground.
		if ((input_vel-md.foot_vel_target[axis_no]<0) != (md.foot_vel_target[axis_no]<0))
		{
			// The sign of the change of velocity is opposite of the sign of velocity,
			// that means we are breaking.
			// It's likely we got to this high speed because the wheels lost contact with the ground.
			// As soon as we regain contact, we want to get to normal speed
			// very quickly, so we don't slow down the breaking acceleration in this case.
			// See monitor_log_jump81.bin history_plot_pos = 1.8f why this is here.
			md.foot_control_limit_accel = false;
		}
	}
	if (md.foot_control_limit_accel)
	{
		// We limit the acceleration, otherwise it can get so big that
		// backcalculation in balance_control doesn't really work anymore when
		// the max_current is 20A or bigger.
		// Not sure why this is the case, maybe the mainloop is too slow.
		input_vel = md.foot_vel_target[axis_no] + clamp(input_vel-md.foot_vel_target[axis_no],
				-cd.foot_control_accel_limit*md.delta_time,
				 cd.foot_control_accel_limit*md.delta_time);
	}

	input_vel = clamp(input_vel, -cd.foot_control_vel_limit, cd.foot_control_vel_limit);
	axis("controller")("input_vel").set(input_vel);
	md.foot_vel_target[axis_no] = input_vel;

	//axis("encoder")("vel_estimate").get(md.foot_vel[axis_no]);
	//axis("encoder")("shadow_count").get(md.foot_shadow_count[axis_no]);
	//axis("encoder")("count_in_cpr").get(md.foot_count_in_cpr[axis_no]);

	//axis("motor")("current_control")("Iq_measured").get(md.foot_current[axis_no]);
	axis("motor")("current_control")("Iq_setpoint").get(md.foot_current_target[axis_no]);
	
	float temperature = 0;
	//if (md.jump_phase != JP_Jumping) axis("fet_thermistor")("temperature").get(temperature);
	//axis("fet_thermistor")("temperature").get(temperature);
	return temperature;
}

bool foot_control_update_1()
{
	u32_micros start_time = time_micros();
	static u32_micros last_time = 0;
	if (cd.enable_foot_control)
	{
		Endpoint& axis0 = odrv_foot.root("axis0");
		Endpoint& axis1 = odrv_foot.root("axis1");

		bool want_torque_mode = false;
		if (md.jump_phase == JP_Jumping)
			want_torque_mode = true;
		if (cd.foot_control_force_torque_mode)
			want_torque_mode = true;
		if (want_torque_mode != is_in_torque_mode)
		{
			is_in_torque_mode = want_torque_mode;
			axis0("controller")("config")("control_mode").set(want_torque_mode ? CONTROL_MODE_TORQUE_CONTROL : CONTROL_MODE_VELOCITY_CONTROL);
			axis1("controller")("config")("control_mode").set(want_torque_mode ? CONTROL_MODE_TORQUE_CONTROL : CONTROL_MODE_VELOCITY_CONTROL);
		}


		// In case there is some delay and delta_time is larger than expected,
		// it is important that we use the real value here. Otherwise backcalculation will
		// add a hickup in the motion.
		//float delta_time_unclamped = float(md.delta_time_micros) * .000001f;
		float delta_time_unclamped = float(start_time-last_time) * .000001f;

		float old_foot_vel_coarse0 = md.foot_vel_coarse[0];
		float old_foot_vel_coarse1 = md.foot_vel_coarse[1];

		float old_pos = md.foot_pos[0];
		axis0("encoder")("pos_estimate").get(md.foot_pos[0]);
		md.foot_vel_coarse[0] = (md.foot_pos[0]-old_pos) / delta_time_unclamped;

		old_pos = md.foot_pos[1];
		axis1("encoder")("pos_estimate").get(md.foot_pos[1]);
		md.foot_vel_coarse[1] = (md.foot_pos[1]-old_pos) / delta_time_unclamped;

		// Calculate foot acceleration in forward direction, in g-force.
		// This is just for debugging. The value is not used in any calculation.
		float foot_acc0 = (md.foot_vel_coarse[0]-old_foot_vel_coarse0) / md.delta_time;
		float foot_acc1 = (md.foot_vel_coarse[1]-old_foot_vel_coarse1) / md.delta_time;
		//md.foot_acc = (foot_acc0 - foot_acc1) * 0.5f * tau * wheel_radius / 1000 / earth_acc;
		md.foot_acc = update_smooth(
				md.foot_acc,
				(foot_acc0 - foot_acc1) * 0.5f * tau * wheel_radius / 1000 / earth_acc,
				cd.foot_acc_smooth_factor, cd.target_delta_time_ms);
	}
	last_time = start_time;
	md.delta_time_foot_control_1 = time_micros() - start_time;
	return true;
}

void foot_control_handle_z_search()
{
	Endpoint& axis0 = odrv_foot.root("axis0");
	Endpoint& axis1 = odrv_foot.root("axis1");
	if (!md.foot_motor_ready[0])
	{
		if (axis0("encoder")("is_ready").get2<bool>() &&
			axis0("encoder")("index_found").get2<bool>())
			md.foot_motor_ready[0] = true;
	}
	if (!md.foot_motor_ready[1])
	{
		if (axis1("encoder")("is_ready").get2<bool>() &&
			axis1("encoder")("index_found").get2<bool>())
			md.foot_motor_ready[1] = true;
	}

	bool do_trigger_z_search = false;

	static int last_encoder_z_search_trigger;
	if (cd.encoder_z_search_trigger-last_encoder_z_search_trigger == 1)
	{
		do_trigger_z_search = true;
		md.foot_motor_ready[0] = false;
		md.foot_motor_ready[1] = false;
	}
	last_encoder_z_search_trigger = cd.encoder_z_search_trigger;

	if (md.startup_sequence == SS_FootIndex1)
	{
		if (md.foot_motor_ready[0] && md.foot_motor_ready[1])
		{
			// Index already found, skip index search.
			md.startup_sequence = SS_StartBalancing1;
		}
		else
		{
			do_trigger_z_search = true;
			md.startup_sequence = SS_FootIndex2;
			md.startup_sequence_timer = 0;
		}
	}
	if (md.startup_sequence == SS_FootIndex2)
	{
		if (md.foot_motor_ready[0] && md.foot_motor_ready[1])
		{
			md.startup_sequence = SS_StartBalancing1;
		}
		else if (md.startup_sequence_timer > 3)
		{
			printf("startup sequence: finding foot encoder index failed!\n");
			md.startup_sequence = SS_Failed;
		}
	}

	if (do_trigger_z_search)
	{
		if (!md.foot_motor_ready[0])
		{
			axis0("requested_state").set(AXIS_STATE_ENCODER_INDEX_SEARCH);
			//printf("trigger axis0 ENCODER_INDEX_SEARCH\n");
		}
		if (!md.foot_motor_ready[1])
		{
			axis1("requested_state").set(AXIS_STATE_ENCODER_INDEX_SEARCH);
			//printf("trigger axis1 ENCODER_INDEX_SEARCH\n");
		}
	}
}

bool foot_control_update_2()
{
	u32_micros start_time = time_micros();

	Endpoint& axis0 = odrv_foot.root("axis0");
	Endpoint& axis1 = odrv_foot.root("axis1");
	
	static int last_stop_startup_sequence;
	if (cd.stop_startup_sequence_trigger-last_stop_startup_sequence == 1)
	{
		if (md.startup_sequence == SS_FootIndex2)
		{
			axis0("requested_state").set(AXIS_STATE_IDLE);
			axis1("requested_state").set(AXIS_STATE_IDLE);
		}
		if (md.startup_sequence != SS_None)
			printf("startup sequence manually canceled.\n");
		md.startup_sequence = SS_None;
		md.jump_phase = JP_None;
		command_reset();
	}
	last_stop_startup_sequence = cd.stop_startup_sequence_trigger;


	if (cd.enable_foot_control)
	{
		foot_control_handle_z_search();
		float temperature0 = foot_control_update_axis(axis0, 0);
		float temperature1 = foot_control_update_axis(axis1, 1);
		md.odrv_foot_temperature = std::max(temperature0, temperature1);
		
		switch (md.counter%17)
		{
		// The first cases are handled in leg_control_update!
		case 12: odrv_foot.root("ibus").get(md.foot_control_bus_current); break;
		case 13: axis0("encoder")("index_check_cumulative_error").get(md.foot_sensor_index_error[0]); break;
		case 14: axis1("encoder")("index_check_cumulative_error").get(md.foot_sensor_index_error[1]); break;
		case 15: axis0("encoder")("index_check_index_count"     ).get(md.foot_sensor_index_count[0]); break;
		case 16: axis1("encoder")("index_check_index_count"     ).get(md.foot_sensor_index_count[1]); break;
		}

		cd_counter = cd.odrive_init_counter;
	}
	else
	{
		if (motor_running[0])
		{
			axis0("requested_state").set(AXIS_STATE_IDLE);
			motor_running[0] = false;
		}
		if (motor_running[1])
		{
			axis1("requested_state").set(AXIS_STATE_IDLE);
			motor_running[1] = false;
		}
		md.foot_vel[0] = 0;
		md.foot_vel[1] = 0;
		md.foot_vel_coarse[0] = 0;
		md.foot_vel_coarse[1] = 0;
		md.foot_current[0] = 0;
		md.foot_current[1] = 0;
		md.foot_current_target[0] = 0;
		md.foot_current_target[1] = 0;
	}
	if (!check_errors(odrv_foot.root))
		return false;

	md.foot_control_endpoint_request_count = odrv_foot.endpoint_request_counter;
	odrv_foot.endpoint_request_counter = 0;
	
	md.delta_time_foot_control_2 = time_micros() - start_time;

	return true;
}
