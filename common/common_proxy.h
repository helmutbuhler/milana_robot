
const int monitor_axes = 2;
const char*const axis_names[monitor_axes] = {"axis0", "axis1"};
const int monitor_data_version = 1;
struct MonitorDataAxis
{
	bool is_running = false;
	bool encoder_ready = false;
	bool motor_is_calibrated = false;
	bool anticogging_valid = false;
	float pos = 0, input_pos = 0;
	float vel = 0, vel_coarse = 0, input_vel = 0;
	float integrator = 0;
	float input_torque = 0;
	float current_target = 0;

	int encoder_shadow_count = 0;
	bool encoder_index_enabled = false;
	int encoder_index_error = 0;
	int encoder_index_count = 0;
};

// This is sent from the proxy to the controller for logging purposes.
// This data structure is serialized in logging files on the client side, which means that new members
// must be appended at the end. Otherwise old files cannot be opened anymore.
// Unused variables are marked as such and cannot be removed for the same reason.
struct MonitorData
{
	int counter = 0; // Increased once per frame on proxy (around 60Hz, see target_delta_time_ms)

	// timing
	u64 uptime_micros = 0;
	time_t local_time = 0;
	int odrive_counter = 0; // Increased on ODrive with 8kHz
	float delta_time = 0; // seconds
	u32_micros delta_time_odrive = 0;
	u32_micros delta_time_sleep = 0;
	u32_micros delta_time_network = 0;

	// odrive oscilloscope
	int oscilloscope_state = 0; // 0: off 1: recording 2: recording done 3: transmitting
	int oscilloscope_start = 0, oscilloscope_end = 0;
	float oscilloscope_transmitting[oscilloscope_transmitting_size] = {0};

	float odrive_bus_voltage = 0;
	float odrive_bus_current = 0;

	u64 odrive_serial_number = 0;    
	u8 odrive_hw_version_major = 0;  
	u8 odrive_hw_version_minor = 0;  
	u8 odrive_hw_version_variant = 0;
	int odrive_fw_version = 0;
	bool odrive_fw_is_milana = false;


	MonitorDataAxis axes[monitor_axes];
};

// ControlData below is mainly used to adjust the value of variables, but it is also used
// to trigger certain functions in the proxy (like starting encoder z search).
// On the control_ui side, this is done by incrementing this trigger variable.
// On the proxy side, these trigger variables are checked every frame, and when it sees that
// the variable increased from the last frame, it triggers the code. This looks something like this:
// 
// static int last_reset_leg_sensors = 0;
// if (cd.reset_leg_sensors_trigger-last_reset_leg_sensors == 1)
// {
//     reset_leg_base();
// }
// last_reset_leg_sensors = cd.reset_leg_sensors_trigger;
// 
// The macro below ensures that the trigger variable in control_ui is initialized to 2 and in
// proxy to 0. Why not initialize both to 0?
// Because when the proxy connects for the first time with the client,
// the trigger variable on the client (control_ui) may already be 1 (if the trigger was used
// in a previous connection). If the static variable would be initialized to 0, the trigger would then
// execute in the first frame the client is connected. Initializing it to 1 doesn't work for a similar reason,
// so it is 2.
// Well right now this isn't necessary anymore because the proxy sends the ControlData when it connects to
// control_ui. But I'm keeping this in just in case soneone wants to cut that out of the network protocol.
#ifdef COMPILING_CONTROL_UI
// control_ui
#define TRIGGER_VAR(x) int x = 2
#else
// proxy
#define TRIGGER_VAR(x) int x = 0
#endif

struct ControlDataAxis
{
	bool enable_axis = true;
	bool enable_motor = false;

	float input_pos = 0;
	float input_vel = 0;
	float input_torque = 0;

	int odrive_set_control_counter = 0; // Increased for odrive axis related changes

	// motor
	bool motor_pre_calibrated = false;
	int pole_pairs = 0;
	float torque_constant = 1;
	float current_lim = 10;
	float current_lim_margin = 8;
	float requested_current_range = 60;

	// controller
	int control_mode = 3;//CONTROL_MODE_POSITION_CONTROL
	int input_mode = 0;//INPUT_MODE_INACTIVE
	float pos_gain = 0;
	float vel_gain = 0;
	float vel_integrator_gain = 0;
	float vel_limit = 10;
	float vel_limit_tolerance = 1.5f;
	float input_filter_bandwidth = 20;
	bool enable_anticogging = false;
	bool enable_vel_limit = true;
	bool enable_overspeed_error = true;

	// encoder
	int encoder_mode = 0; // ENCODER_MODE_INCREMENTAL
	bool encoder_use_index = false;
	bool encoder_pre_calibrated = false;
	int encoder_cpr = 0;
	float encoder_bandwidth = 1000;
    int encoder_abs_spi_cs_gpio_pin = 1;
    bool encoder_ignore_abs_ams_error_flag = false;

	TRIGGER_VAR(encoder_z_search_trigger);
	TRIGGER_VAR(calibration_trigger);
};

// This is sent to the proxy to control some variables from control_ui.
// Note: These variables are never changed by the proxy itself. If control_ui
// never connects, all these variables are basically constants.
// The initial state of these variables is sent from the proxy to control_ui
// when control_ui connects so that the ui can show the correct values.
// After that, control_ui sends these variables to the proxy everytime the user changes
// one of them.
struct ControlData
{
	int counter = 0; // this is increased when one of these variables change
	
	// timing:
	int target_delta_time_ms = 10;

	int odrive_set_control_counter = 0; // Increased for odrive related changes

	float max_regen_current = 0.0f;
    float brake_resistance = 2.0f;     // [ohm]
    float dc_max_positive_current = INFINITY; // Max current [A] the power supply can source
    float dc_max_negative_current = -0.000001f; // Max current [A] the power supply can sink. You most likely want a non-positive value here. Set to -INFINITY to disable.
    int uart_baudrate = 115200;
	float ibus_report_filter_k = 0.001f;
	bool generate_error_on_filtered_ibus = true;

	bool stop_motors_on_disconnect = true; // stop motors when control_ui is no longer connected

	// oscilloscope
	TRIGGER_VAR(oscilloscope_force_trigger);
	
	TRIGGER_VAR(odrive_save_configuration_trigger);
	TRIGGER_VAR(odrive_reboot_trigger);
	
	ControlDataAxis axes[monitor_axes];
};
