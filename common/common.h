// This file includes common data structures that are used for communication between the robot and control_ui.
#pragma once
#include "helper.h"
#include "time_helper.h"

const u16 port = 60123; // For connection between robot and control_ui.
const u16 tts_port = 60124;
const u16 asr_port = 60125;

// Update rate on ODrive
const int odrive_frequency = 8000; //Hz
const int foot_cpr = 8192;
const int hip_cpr = 8192;


const float earth_acc = 9.80665f;

const int oscilloscope_transmitting_size = 64;
const int imu_fifo_gyro_max_count = 16;

const int num_arm_servos = 6;

// Some CAD specifications, lengths are all in mm.
const float leg_gear_reduction = 3;
//const float hip_angle_1 = to_rad(80);
//const float hip_angle_0_ = to_rad(-30); // according to CAD
const float hip_angle_0 = to_rad(-25); // according to measurement
const float leg_angle_1 = to_rad(-130);
const float wheel_radius = 47;
const float upper_leg_length = 125;
const float lower_leg_length = 91.3685f;

// ODrive connection
//const int odrive_uart_baudrate = 115200 ;
//const int odrive_uart_baudrate = 230400 ;
//const int odrive_uart_baudrate = 500000 ; // invalid stream packet 2! 2 1
const int odrive_uart_baudrate = 921600 ; // works with 2 stopbits
//const int odrive_uart_baudrate = 1152000;
//const int odrive_uart_baudrate = 1500000;
//const int odrive_uart_baudrate = 2000000; // doesnt work

const bool odrive_uart_stop_bits_2 = true;


enum JumpPhase
{
	JP_None,
	JP_Positioning,
	JP_Jumping,
	JP_LegUp,
	JP_Falling,
	JP_Landing,
	JP_PostLanding,
};

enum StartupSequence
{
	SS_None,
	SS_Failed,
	SS_LegCalibration1a,
	SS_LegCalibration1b,
	SS_Hip1,
	SS_Hip2,
	SS_ImuCalibration1,
	SS_ImuCalibration2,
	SS_LegCalibration2a,
	SS_LegCalibration2b,
	SS_Hip3,
	SS_FootIndex1,
	SS_FootIndex2,
	SS_StartBalancing1,
	SS_StartBalancing2,
	SS_FinalLegHipMovement,
};

// Things to say to test tts output
const int tts_say_things_num = 3;
char const*const tts_say_things[tts_say_things_num] =
{
	"The first manned Moon landing was Apollo 11 on July, 20 1969. The first human to step on the Moon was astronaut Neil Armstrong followed second by Buzz Aldrin. They landed in the Sea of Tranquility with their lunar module the Eagle. They were on the lunar surface for 2.25 hours and collected 50 pounds of moon rocks.",
	"To alcohol! The cause of, and solution to, all of life's problems",
	"I don't say this and i don't say that. Because when i would say this and it is that and i would say that and it is this then i would say this and that.",
};

static_assert(sizeof(time_t) == 8, "");

// This is sent from the robot to the controller for logging purposes.
// This data structure is serialized in logging files on the client side, which means that new members
// must be appended at the end. Otherwise old files cannot be opened anymore.
// Unused variables are marked as such and cannot be removed for the same reason.
struct MonitorData
{
	int counter = 0; // Increased once per frame on robot (around 60Hz, see target_delta_time_ms)

	// timing
	int odrive_counter = 0; // Increased on ODrive with 8kHz
	float delta_time = 0; // seconds
	u32_micros delta_time_micros = 0;
	u32_micros delta_time_battery = 0;
	u32_micros delta_time_leg_control = 0;
	u32_micros delta_time_sleep = 0;
	u32_micros delta_time_network = 0;
	u32_micros delta_time_imu = 0;
	u32_micros delta_time_joystick = 0;
	u32_micros delta_time_servo = 0;
	u32_micros delta_time_foot_control_1 = 0;
	u32_micros delta_time_foot_control_2 = 0;

	// battery
	float battery_total_voltage = 0;
	float battery_1_voltage = 0;
	float battery_1_sensor_voltage = 0;
	int battery_sensor_counter = 0;
	float battery_status_1 = 0, battery_status_2 = 0;

	// temperature
	float jetson_temperature = 0;
	float odrive_temperature = 0;
	float imu_temperature = 0;
	float odrv_foot_temperature = 0;

	// leg control:
	// all "angles" here are in revolutions of the small gear
	// 0: right  1: left
	// initial absolute value: 0/right: -1; 1/left: 1
	float leg_rel_angle[2] = {0}; // relative angle
	float leg_base_angle[2] = {0}; // base angle (base+relative = absolute)
	float leg_rel_angle_target[2] = {0};
	float leg_rel_angle_target_odrv[2] = {0};
	float leg_encoder_error_rate[2] = {0};
	bool leg_error_flag[2] = {false, false};
	float leg_vel[2] = {0};  // velocity, revolutions per second,
	float leg_integrator[2] = {0}; // vel_integrator_torque
	float leg_current[2] = {0};
	float leg_current_target[2] = {0};
	float common_leg_pos_target = 0;
	float common_leg_pos_sensor = 0;
	int leg_control_debug1 = 0;
	int leg_control_debug2 = 0;
	int leg_control_debug3 = 0;
	float leg_control_pos_gain_check[2] = {0,0};
	float average_leg_pos_integrator = 0;

	// odrive oscilloscope
	int oscilloscope_state = 0; // 0: off 1: recording 2: recording done 3: transmitting
	int oscilloscope_start = 0, oscilloscope_end = 0;
	float oscilloscope_transmitting[oscilloscope_transmitting_size] = {0};
	
	// positioning
	float avg_error_p = 0;
	bool positioning_right_phase_ = false; // unused

	// jumping
	JumpPhase jump_phase = JP_None;
	float jump_timer = 0;

	// falling
	bool enabled_landing_detector = false;
	bool force_stay_in_falling_mode = false;

	// landing
	bool activated_landing_[2] = {0, 0}; // unused
	float post_landing_timer = 1;

	// side balancing
	float post_ramp_timer = 1;
	float side_angle_target = 0;
	float side_balance_error = 0;
	float side_balance_error_d = 0;
	float side_angle_hip_correction = 0;
	bool side_balance_recover_mode = false;
	float side_balance_recover_mode_timer = 0;

	// foot control:
	// all "angles" here are in revolutions of the foot wheel
	// 0: left  1: right
	float foot_pos[2] = {0};
	float foot_vel[2] = {0}; // velocity, calculated by odrive
	float foot_vel_coarse[2] = {0}; // velocity, calculated by differentiating foot_pos in the main loop.
	float foot_vel_target[2] = {0};
	float foot_current[2] = {0};
	float foot_current_target[2] = {0};
	float foot_set_current_lim[2] = {0, 0};
	bool foot_motor_ready[2] = {false, false}; // has the z-index been found yet?
	//int foot_count_in_cpr[2] = {0, 0};

	// hip
	float common_hip_pos_target = 0.75f;
	float common_hip_pos_sensor = 0.75f;
	float common_servo_vel = 0;
	float hip_balance_current = 0;
	float hip_balance_target = 0;
	float hip_pos_target[2] = {0, 0};
	float servo_pos[2] = {0, 0};
	float hip_sensor_correction_delta[2] = {0, 0};
	int hip_sensor_raw1[2] = {0};
	int hip_sensor_raw2[2] = {0};
	float hip_sensor_pos_relative[2] = {0};
	float hip_sensor_pos_base[2] = {0};
	float hip_sensor_pos[2] = {0};
	float hip_sensor_vel[2] = {0};

	// imu
	float imu_x_angle = 0;
	float imu_side_angle = 0;
	float imu_gyro_bias_x = 0;
	float imu_gyro_bias_y = 0;
	float imu_gyro_bias_z = 0;
	// Measured imu acceleration, in g-force:
	float imu_gravity_acc_x = 0;
	float imu_gravity_acc_y = 0;
	float imu_gravity_acc_z = 0;
	// Estimated gravity vector, based on accelerometer, gyroscope and body acceleration. Normalized:
	float imu_gravity_x = 0;
	float imu_gravity_y = 0;
	float imu_gravity_z = 0;
	// Measured gyroscope data:
	float imu_gyro_x = 0;
	float imu_gyro_y = 0;
	float imu_gyro_z = 0;

	bool imu_do_bias_estimation = false;
	float imu_bias_estimation_timer = 0;
	int imu_bias_estimation_counter = 0;
	float imu_bias_estimation_gyro_sum_x = 0;
	float imu_bias_estimation_gyro_sum_y = 0;
	float imu_bias_estimation_gyro_sum_z = 0;
	int imu_fifo_gyro_count = 0;
	float imu_fifo_gyro_x[imu_fifo_gyro_max_count] = {0};
	float imu_fifo_gyro_y[imu_fifo_gyro_max_count] = {0};
	float imu_fifo_gyro_z[imu_fifo_gyro_max_count] = {0};

	// balance control
	float error_p = 0; // pid error
	float error_d = 0;
	float vel_p = 0; // pid contributions to balance_control_vel_output
	float vel_pos = 0;
	float balance_control_integrator = 0;
	float vel_backcalculation = 0;
	float balance_control_standing_action_tolerance_timer = 0;
	float t_correction_angle = 0;
	float balance_control_body_action_ = 0; // unused
	float balance_control_body_action_delta_ = 0; // unused
	bool is_standing = false;
	float balance_control_pos_user_ = 0; // unused
	float balance_control_pos_output_ = 0; // unused
	float balance_control_vel_output = 0; // target velocity
	float balance_control_vel_output_smooth = 0;
	float balance_control_vel_user = 0;
	float balance_control_vel_quasi = 0;
	float balance_control_vel_quasi_smooth = 0;
	float balance_control_rotation_error_bias = 0;
	float balance_control_rotation_error = 0;
	float balance_control_rotation_vel = 0;
	float balance_control_vel_extra_factor = 1.0f;
	float balance_control_delta_pos_l = 0;
	float balance_control_delta_pos_r = 0;
	float balance_control_delta_vel_l = 0;
	float balance_control_delta_vel_r = 0;
	
	// startup
	StartupSequence startup_sequence = SS_None;
	float startup_sequence_timer = 0;

	// to be ordered:
	bool balance_control_force_still = false;
	int monitor_data_version = 31;
	float balance_control_gain_p = 0;
	float balance_control_gain_i = 0;
	float balance_control_gain_vel_high = 0;
	float balance_control_gain_backcalculation = 0;
	float foot_acc = 0; // in g-force
	// Measured imu acceleration, with body acceleration subtracted:
	float imu_gravity_acc_x_comp = 0;
	float imu_gravity_acc_y_comp = 0;
	float imu_gravity_acc_z_comp = 0;
	float imu_sensor_to_floor_pos_x = 0; // x is forward direction
	float imu_sensor_to_floor_pos_y = 0; // y is side direction
	float imu_sensor_to_floor_vel_x = 0;
	float imu_sensor_to_floor_vel_y = 0;
	float imu_sensor_to_floor_acc_x = 0; // in g-force
	float imu_sensor_to_floor_acc_y = 0; // in g-force
	float vel_d;
	float balance_control_gain_d = 0;
	float balance_control_gain_vel_low = 0;
	float balance_control_squat_length = 0;
	float common_leg_vel_target = 0;
	int hip_sensor_index_error[2] = {0};
	int hip_sensor_index_count[2] = {0};
	u64 uptime_jetson_micros = 0;
	time_t local_time = 0;
	bool balance_control_is_quiet = false;
	float balance_control_target_angle = 0;
	float balance_control_target_angle_vel = 0;
	float leg_control_bus_current = 0;
	float foot_control_bus_current = 0;
	float l_base_angle = 0;
	float leg_vel_target_odrv[2] = {0};
	int leg_control_endpoint_request_count = 0;
	int foot_control_endpoint_request_count = 0;
	float l_frame_counter[2] = {0};
	float leg_pos_target_ik[2] = {0};
	float hip_pos_target_ik[2] = {0};
	float joystick_input_timer = 0;
	float leaning_target_angle_joystick = 0;
	float leaning_target_angle = 0;
	float leaning_target_angle_smooth = 0;
	float leaning_optimized_target_angle = 0;
	float leaning_min_rotation_vel = -1000;
	float leaning_max_rotation_vel = 1000;
	float balance_control_world_angle = 0;
	float cpu_usage[4] = {-1,-1,-1,-1};
	int ram_avail_kb = -1;
	int swap_avail_kb = -1;
	bool foot_control_limit_accel = false;
	bool tts_client_connected = false;
	bool asr_client_connected = false;
	float speech_service_ping_timer_ = 2; // unused
	u32_micros delta_time_tts = 0;
	u32_micros delta_time_asr = 0;
	bool tts_calculating = false;
	bool tts_playback = false;

	int command_running = -1;
	float command_timer = 0;

	float ik_x_delta_target = 0;
	float ik_x_delta = 0;
	//float ik_x_delta_realized = 0;
	float balance_control_position_control_target = 0;
	float ik_l_arm_a = to_rad(40), ik_l_arm_b = 0, ik_l_arm_c = 0;
	float ik_r_arm_a = to_rad(40), ik_r_arm_b = 0, ik_r_arm_c = 0;
	float ik_arm_target_x = 180, ik_arm_target_y = 80, ik_arm_target_z = -140;

	float arm_pos_target[num_arm_servos] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
	float arm_pos_target_smooth[num_arm_servos] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
	u32_micros delta_time_ik = 0;
	bool ik_enable_arm_mode = false;

	bool hip_sensor_index_enabled = false;

	float imu_temp_gyro_x = 0;
	float imu_temp_gyro_y = 0;
	float imu_temp_gyro_z = 0;

	int foot_sensor_index_error[2] = {0};
	int foot_sensor_index_count[2] = {0};

	int ik_winking_phase = 0;

	float coi_world_pos = 0; // center of inertia in forward direction in world
	float coi_world_vel = 0;
	float coi_world_acc = 0; // in g-force
	float t_correction_angle_raw = 0;
	float coi_theta_acc = 0; // in g-force
	
	int foot_shadow_count[2] = {0, 0};

	bool tts_recv_audio = false;
	bool asr_recv_text = false;
	int tts_queue_size = 0;
	int command_queue_size = 0;
};

// ControlData below is mainly used to adjust the value of variables, but it is also used
// to trigger certain functions in the robot (like starting encoder z search).
// On the control_ui side, this is done by incrementing this trigger variable.
// On the robot side, these trigger variables are checked every frame, and when it sees that
// the variable increased from the last frame, it triggers the code. This looks something like this:
// 
// static int last_reset_leg_sensors = 0;
// if (cd.reset_leg_sensors_trigger-last_reset_leg_sensors == 1)
// {
//     reset_leg_base();
// }
// last_reset_leg_sensors = cd.reset_leg_sensors_trigger;
// 
// The macro below ensures that the trigger variable in control_ui is initialized to 2. Why is that?
// Because when the robot connects for the first time with the client,
// the trigger variable on the client (control_ui) may already be 1 (if the trigger was used
// in a previous connection). If the static variable would be initialized to 0, the trigger would then
// execute in the first frame the client is connected. Initializing it to 1 doesn't work for a similar reason,
// so it is 2.
#ifdef COMPILING_CONTROL_UI
// control_ui
#define TRIGGER_VAR(x) int x = 2
#else
// robot
#define TRIGGER_VAR(x) int x = 0
#endif

// This is sent to the robot to control the movement and adjust variables
// Note: These variables are never changed by the robot itself. If the client
// never connects, all these variables are basically constants.
struct ControlData
{
	int counter = 0; // this is increased when one of these variables change
	
	// timing:
	int target_delta_time_ms = 18;
	int max_delta_time_ms = 18;
	int pause_delta_time_ms = 200;
	TRIGGER_VAR(pause_trigger);
	bool do_random_pauses = false;
	int random_pauses_max_ms = 60;

	// leg control:
	bool enable_leg_control = true;
	int odrive_init_counter = 0; // Increased for odrive axis state values
	float bus_current_smooth = 0.001f;
	bool generate_error_on_filtered_ibus = true;
	float current_lim = 15;
	float current_lim_margin = 20;
	bool leg_control_enable_motor[2] = {false, false};
	float leg_abs_target[2] = {0,0};
	//int input_mode = 1;//INPUT_MODE_PASSTHROUGH
	int input_mode = 3;//INPUT_MODE_POS_FILTER
	float vel_gain = 3; float pos_gain = 30; float vel_integrator_gain = 5; // less em noise (less speaker disturbance)
	//float vel_gain = 13; float pos_gain = 10; float vel_integrator_gain = 30; // normal
	//float vel_gain = 2; float pos_gain = 3; float vel_integrator_gain = 5; // very old sidebalanceV1
	//float vel_gain = 2; float pos_gain = 50; float vel_integrator_gain = 2; // very old sidebalanceV2
	float vel_limit = 10;
	float accel_limit = 5;
	float input_filter_bandwidth = 20;
	float encoder_bandwidth = 1000;
	TRIGGER_VAR(reset_leg_sensors_trigger);

	// oscilloscope
	TRIGGER_VAR(oscilloscope_force_trigger);

	// stand control
	bool enable_stand_control = false;
	float common_leg_vel_target = 0;
	float common_leg_vel_user_max = 1.3f;
	float common_leg_pos_min = -0.65f, common_leg_pos_max = 0.97f;
	float common_leg_pos_min_side_balance = 0.6f;
	
	// side balance
	bool enable_side_balance = false;
	float side_angle_target = 0;
	float side_gain_p = 100;
	float side_gain_d = 0;
	float average_gain_p = 100;
	float average_gain_i = 15;
	float average_gain_d = 3;
	float target_gain_p = 50;
	float target_gain_d = 3.0;
	float side_angle_motor_force_limit = 20;
	int side_balance_mode = 1;
	float side_angle_hip_correction_factor = 0.5f;
	float side_balance_down_limit_factor = 10;
	float side_balance_down_limit_threshold = -6;
	float side_balance_max_vertical_angle = -0.35f;
	float side_balance_max_vertical_angle_gain = 20;
	float side_balance_recover_mode_side_angle_threshold_on = 0.05f;
	float side_balance_recover_mode_side_angle_threshold_off = 0.04f;
	float side_balance_recover_mode_side_angle_time_on = 0.4f;
	float side_balance_recover_mode_side_angle_time_off = 0.7f;
	float side_balance_recover_mode_side_angle_delta = 0.1f;
	bool start_oscilloscope_on_ramp = false;
    float acc_smooth_factor = 0.999f;
    float acc_current_gain = 1200;
    float side_balance_error_d_smooth_factor = 0.9876797f;

	// positioning
	bool do_positioning = true;
	float positioning_error_p_threshold = 0.01f;
	float positioning_vel_delta = -0.05f;

	// jumping
	TRIGGER_VAR(jump_phase_trigger);
	bool start_oscilloscope_on_jump = false;
	float jump_pos_target = 0;
	float j_vel_gain = 10;
	float j_pos_gain = 60;
	float j_vel_integrator_gain = 0;
	float j_vel_limit = 100;
	float j_current_lim = 50;
	float j_current_lim_margin = 100;
	float jump_current_side_deviation = -0.3f*0;
    int jump_left_right_timeout = 0; // delay jump on one leg in no. of odrive frames. Sign decides which leg.

	// leg up
	bool do_jump_leg_up = true;
	float leg_up_pos_target = 0.9f;
	float leg_up_min_time = 0.2f;
	//float leg_up_input_filter_bandwidth = 50;

	// falling
	TRIGGER_VAR(fall_phase_trigger);
	float fall_min_leg_angle = 0.6f, fall_max_leg_angle = 0.9f;
	float fall_vertical_angle = -0.3f;
	float landing_detector_current_threshold = -6.0f;
	float landing_detector_delay = 0.25f;
	int f_input_mode = 3;//INPUT_MODE_POS_FILTER
	float f_vel_gain = 4;
	float f_pos_gain = 50;
	float f_vel_integrator_gain = 0;
	float f_vel_limit = 8;
	float f_input_filter_bandwidth = 50;
	float f_current_lim = 30; // current limit for falling and landing phase
	float fall_servo_correction = 2;
	float fall_servo_target = 0.3f;
	float fall_servo_x_angle_stop = -10;//0.24f;

	// landing
	float l_leg_pos_target = 0.95f;
    float l_base_angle_delta = -0.4f;
    float l_max_time = 0.2f;
    float l_vel_gain = 15;//20
    float l_vel_integrator_gain = 0;
    float l_sync_gain_pos = 30;
    float l_sync_gain_vel = 1;
	float landing_foot_vel_delta_time = 0.15f;
	float landing_foot_vel_delta = 1.5f;
	float initial_integrator_after_landing = -6;
	float l_initial_measure_time = 0.03f*0;
    float l_initial_measure_current = -5;
    bool l_do_max_vel = false;
    bool l_do_straightening = false;
    float l_initial_expected_accel_per_s = 600;//200
    float l_initial_accel_gain = 20;//40
    float l_pos_target_delta_per_s = 10; // target position difference per seconds delay of second landing foot

	// post landing
	float post_landing_time = 0.2f;

	// foot control:
	bool enable_foot_control = true;
	bool foot_control_enable_motor[2] = {false, false};
	float foot_control_current_lim = 30;
	float foot_control_current_lim_landing = 30;
	float foot_control_current_lim_margin = 20;
	float foot_control_vel_target[2] = {0,0};
	float foot_control_vel_gain = 8;
	float foot_control_vel_integrator_gain = 300;
	float foot_control_vel_limit = 15;
	float foot_control_accel_limit = 50;
	TRIGGER_VAR(encoder_z_search_trigger);
	bool foot_control_enable_anticogging = true;
	float foot_acc_smooth_factor = 0.94f;
	bool foot_control_force_torque_mode = false;
	float foot_control_torque_target[2] = {-0.3f, -0.3f};

	// servo:
	bool servo_enable = false;
	bool servo_manual_enable = false;
	float servo_manual_target[2] = {0.75f, 0.75f};
	int servo_channel[2] = {0, 1};
	u16 servo_min[2] = {191, 524}; //90
	u16 servo_max[2] = {660, 93}; //660
	float common_servo_target_vel_user = 0;
	float common_servo_vel_user_max = 1;
	float servo_vel_max = 2.0f; // todo: calibrate
	float hip_sensor_angle_range = 1.847749f; // todo: rename
	//float wheel_side_distance = 87.0f*2; // distance of the two wheels (according to CAD)
	float wheel_side_distance = 187; // distance of the two wheels (measured)
	bool do_hip_sensor_index = false;
	float hip_sensor_pos_base_manual[2] = {0.830f, 0.814f};
	bool do_hip_sensor_pos_base_manual = true;

	// arm
	// arrays have these six elements: l_arm_a, l_arm_b, l_arm_c, r_arm_a, r_arm_b, r_arm_c
	float arm_manual_target[num_arm_servos] = {0.5f, 0.5f, 0.5f, 0.5f, 0.5f, 0.5f};
	int arm_servo_channel[num_arm_servos] = {6, 4, 5, 8, 10, 11};
	bool arm_servo_v2[num_arm_servos] = {0, 0, 0, 0, 0, 0};
	u16 arm_servo_min = 120;
	u16 arm_servo_max = 610;
	u16 arm_servo_min_v2 = 120;
	u16 arm_servo_max_v2 = 590;
	bool arm_enable_angle_control = false;
	float arm_angle_target[num_arm_servos] = {to_rad(60), to_rad(40), to_rad(20), to_rad(60), to_rad(40), to_rad(20)};
	float l_arm_a_pos_0 = 0.90f, l_arm_a_pos_90 = 0.56f;
	float l_arm_b_pos_0 = 0.58f, l_arm_b_pos_40 = 0.93f;
	float l_arm_c_pos_0 = 0.54f, l_arm_c_pos_40 = 0.21f;
	float r_arm_a_pos_0 = 0.13f, r_arm_a_pos_90 = 0.47f;
	float r_arm_b_pos_0 = 0.49f, r_arm_b_pos_40 = 0.13f;
	float r_arm_c_pos_0 = 0.59f, r_arm_c_pos_40 = 0.93f;
	float arm_a_servo_vel_max = 0.3f;
	float arm_b_servo_vel_max = 2;
	float arm_c_servo_vel_max = 2;

	TRIGGER_VAR(reset_hip_sensors_trigger);
	bool hip_sensor_enable_correction = true;
	float hip_sensor_correction_factor = 3;
	float hip_sensor_correction_dead_zone = 0.01f;
	float hip_sensor_max_deviation = 0.2f;
	float hip_sensor_correction_side_angle_factor = 0.2f;

	// imu
	float imu_gyro_weight = 200; // factor used in complementary filter. The higher, the more weight has the gyroscope over the accelerometer.
	float imu_gyro_scale = 0.01872f;
	float imu_bias_x = -0.006f, imu_bias_y = 0.043f, imu_bias_z = -0.022f; // In case the imu sensor is slightly rotated, this can be adjusted to correct this.
	TRIGGER_VAR(imu_bias_estimation_trigger);
	float imu_bias_estimation_wait_time = 1;
	bool imu_force_no_acc = false;
	bool imu_compensate_body_acc = true;
	int imu_gyro_frequency = 5;
	int imu_fifo_frequency = 5;
	bool imu_use_gyro_thread = true;
	/*float x_angle_from_hip_l_bias = 0;
	float x_angle_from_hip_l_factor = 0;
	float x_angle_from_hip_r_bias = 0;
	float x_angle_from_hip_r_factor = 0;*/
	float imu_sensor_to_floor_vel_smooth_factor = 0.94f;
	float imu_sensor_to_floor_acc_smooth_factor = 0.94f;
	float coi_world_vel_smooth_factor = 0.95f;
	float coi_world_acc_smooth_factor = 0.99f;
	int coi_theta_past = 3;
	float imu_temp_to_gyro_x_a =-0.05150f;
	float imu_temp_to_gyro_x_b = 4.31000f;
	float imu_temp_to_gyro_y_a = 0.02280f;
	float imu_temp_to_gyro_y_b =-5.02000f;
	float imu_temp_to_gyro_z_a = 0.03480f;
	float imu_temp_to_gyro_z_b =-3.32000f;

	// balance control
	bool balance_control_enable = false;
	// pid for proper movement
	float balance_control_gain_p = 10; // 14 if use_coi_vel = false
	float balance_control_gain_i = 60;
	float balance_control_gain_d = 0.2f;
	float balance_control_gain_vel_high = 2.0f; // 3 if use_coi_vel = false
	float balance_control_gain_vel_low = 3.0f; // 5 if use_coi_vel = false
	float balance_control_gain_backcalculation = 40;
	bool balance_control_integrator_rotation_decay = true;
	float balance_control_max_theta_integrator = 0.5f;
	float balance_control_target_angle_vel_gain = 0.02f;
	float balance_control_target_angle_vel_smooth = 0.97f;
	float stepper_avg_smooth = 0.01f;
	float t_correction_simple_factor = 0.005f*0;
	bool do_t_correction_acc = true;
	float t_correction_acc_p = 4;
	float t_correction_angle_max = 0.2f;
	float t_correction_acc_smooth = 0.998f;
	float t_correction_max_rotation_vel = 1;
	float balance_control_vel_user_acceleration = 20;
	bool balance_control_use_coi_vel = true;
	float balance_control_vel_acceleration_limit = 0.25f;
	float balance_control_vel_target = 0;
	bool  balance_control_force_still = false;
	float balance_control_x_delta_vel_max = 0.3f;
	float balance_control_vel_user_max = 1.5f;
	float balance_control_vel_extra_factor_speed = 2.0f;
	float balance_control_vel_extra_factor_max = 5.0f;
	float balance_control_standing_theta_tolerance = 0.8f;
	float balance_control_standing_action_tolerance = 5;
	float balance_control_standing_action_tolerance_time = 1.5f;
	float balance_control_gain_vel_landing_warmup_time = 0;
	
	float balance_control_rotation_vel_user_max = 3;
	float balance_control_rotation_vel_user_target = 0;
	float balance_control_rotation_vel_acc = 4;
	float balance_control_rotation_error_factor = 0.5f;

	float balance_control_error_d_smooth = 0.9f;
	float balance_control_vel_quasi_smooth_factor = 0.999f;
	float balance_control_vel_quasi_smooth_max_acc = 10;

	bool  balance_control_enable_position_control = false;
	float balance_control_position_control_gain_p_high = 1.0f;
	float balance_control_position_control_gain_d_high = 0.3f;
	float balance_control_position_control_gain_p_low = 1.5f;
	float balance_control_position_control_gain_d_low = 0.5f;


	float hip_balance_action_factor = 0.07f*0; // hip_balance_deadzone = 0.06f; hip_balance_integrator = 2.5f;
	float hip_balance_backcalc_factor = 0;
	float hip_balance_theta_factor = 0.4f*0; // hip_balance_deadzone = 0.07f; hip_balance_integrator = 2;
	float hip_balance_x_factor = 0;
	float hip_balance_deadzone = 0.06f;
	float hip_balance_factor = 1;
	float hip_balance_integrator = 2.5f;

	// squat stabilization
	// (masses are in kg, inertia are in kg*m^2, lengths are in mm)
	float squat_body_cog_x = -10.502f;
	float squat_body_cog_y = -93.145f;
	float squat_body_mass = 1.467f;
	float squat_whole_body_inertia = 0.029f; // This is much lower than in the simulation. Don't know why, but this value works better with acc theta calculation.
	float squat_upper_leg_cog_x = 0;
	float squat_upper_leg_cog_y = 57.683f;
	float squat_upper_leg_mass = 1.6f;
	float squat_lower_leg_cog_x = 0;
	float squat_lower_leg_cog_y = 70.384f;
	float squat_lower_leg_mass = 1.06f;
	float squat_wheel_mass = 0;
	float squat_wheel_inertia = 0.001f;
	float squat_base_angle2 = -0.025f;
	float squat_hip_angle_bias = 0;
	float squat_leg_angle_bias = 0;
	float squat_bias_x = 0;
	float squat_bias_y = 0;
	float squat_upper_leg_play_factor = 6.214f;
	float squat_lower_leg_play_factor = 23.368f;

	// leaning
	float leaning_target_angle_user = 0;
	float leaning_target_angle_smooth_factor = 0.995f;
	float leaning_target_angle_max_vel_low = 1.0f;
	float leaning_target_angle_max_vel_high = 0.4f;
	float leaning_max_angle = 0.2f;
	float leaning_angle_rot_tolerance = 0.6f;
	float leaning_vel_tolerance = 0.4f;
	bool leaning_clamp_rotation_vel = true;

	// IK
	int   ik_optimizer_iterations = 100;
	float ik_optimizer_epsilon = 0.0001f;
	float ik_optimizer_lr = 0.0001f;
	float ik_optimizer_x_delta_lr = 0.3f;
	float ik_optimizer_alpha = 0.95f;
	float ik_optimizer_error_angle_factor = 1.0f;
	float ik_optimizer_error_angle_scale = 1.5f;
	float ik_optimizer_error_wheel_x_var_factor = 0.003f;
	float ik_optimizer_error_leaning_angle_factor = 100;
	float ik_optimizer_error_arm_factor = 0.01f;
	float ik_optimizer_error_arm_scale = 2;
	float ik_optimizer_error_x_delta_factor = 0.001f;
	float ik_optimizer_error_collision_factor = 0.4f;
	bool ik_enable = true;
	bool ik_enable_arm_mode = false;
	float ik_x_delta_target = 0;
	float ik_arm_target_x = 180, ik_arm_target_y = 80, ik_arm_target_z = -140;
	int ik_target_counter = 0;
	bool ik_use_thread = true;

	// startup
	TRIGGER_VAR(startup_sequence_trigger);
	TRIGGER_VAR(stop_startup_sequence_trigger);
	float startup_leg_vel = 0.1f;
	float startup_leg_current_threshold = 4;
	float startup_hip1_servo_vel = -2.0f;
	float startup_hip1_servo_vel_threshold = 1.0f;
	float startup_hip1_servo_min = 0.5f;
	float startup_hip2_servo_vel = 0.2f;
	float startup_hip3_servo_vel = 0.4f;
	float startup_balance_control_standing_tolerance = 0.55f;
	float startup_leg_final_pos = 0.737f;
	float startup_hip_final_pos = 0.82f;

	// tts
	float tts_volume = 1.0f;
	int tts_say_thing_index = 0; //if not negative, index into tts_say_things of what to say. if negative, it's a command and not a text to say
	TRIGGER_VAR(tts_say_thing_trigger);
};

