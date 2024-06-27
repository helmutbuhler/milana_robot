// This has some helper functions and constants that come in handy when dealing with ODrive
// but it's all optional.
// The constants are copied from the ODrive repo and are needed because they are not part
// of the ODrive json definition file.
#pragma once
#include "ODrive.h"
#include "endpoint.h"

const int AXIS_STATE_UNDEFINED = 0;           //<! will fall through to idle
const int AXIS_STATE_IDLE = 1;                //<! disable PWM and do nothing
const int AXIS_STATE_STARTUP_SEQUENCE = 2; //<! the actual sequence is defined by the config.startup_... flags
const int AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3;   //<! run all calibration procedures; then idle
const int AXIS_STATE_MOTOR_CALIBRATION = 4;   //<! run motor calibration
const int AXIS_STATE_SENSORLESS_CONTROL = 5;  //<! run sensorless control
const int AXIS_STATE_ENCODER_INDEX_SEARCH = 6; //<! run encoder index search
const int AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7; //<! run encoder offset calibration
const int AXIS_STATE_CLOSED_LOOP_CONTROL = 8;  //<! run closed loop control
const int AXIS_STATE_LOCKIN_SPIN                   = 9;
const int AXIS_STATE_ENCODER_DIR_FIND              = 10;
const int AXIS_STATE_HOMING                        = 11;
const int AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION = 12;
const int AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION = 13;

// from: ODrive/Firmware/autogen/interfaces.hpp
// also here: ODrive/tools/odrive/enums.py
const int AXIS_ERROR_INVALID_STATE               = 0x00000001;
const int AXIS_ERROR_DC_BUS_UNDER_VOLTAGE        = 0x00000002;
const int AXIS_ERROR_DC_BUS_OVER_VOLTAGE         = 0x00000004;
const int AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x00000008;
const int AXIS_ERROR_BRAKE_RESISTOR_DISARMED     = 0x00000010;
const int AXIS_ERROR_MOTOR_DISARMED              = 0x00000020;
const int AXIS_ERROR_MOTOR_FAILED                = 0x00000040;
const int AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x00000080;
const int AXIS_ERROR_ENCODER_FAILED              = 0x00000100;
const int AXIS_ERROR_CONTROLLER_FAILED           = 0x00000200;
const int AXIS_ERROR_POS_CTRL_DURING_SENSORLESS  = 0x00000400;
const int AXIS_ERROR_WATCHDOG_TIMER_EXPIRED      = 0x00000800;
const int AXIS_ERROR_MIN_ENDSTOP_PRESSED         = 0x00001000;
const int AXIS_ERROR_MAX_ENDSTOP_PRESSED         = 0x00002000;
const int AXIS_ERROR_ESTOP_REQUESTED             = 0x00004000;
const int AXIS_ERROR_HOMING_WITHOUT_ENDSTOP      = 0x00020000;
const int AXIS_ERROR_OVER_TEMP                   = 0x00040000;
const int AXIS_ERROR_UNKNOWN_POSITION            = 0x00080000;

const s64 MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE   = 0x000000001;
const s64 MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE   = 0x000000002;
const s64 MOTOR_ERROR_ADC_FAILED                      = 0x000000004;
const s64 MOTOR_ERROR_DRV_FAULT                       = 0x000000008;
const s64 MOTOR_ERROR_CONTROL_DEADLINE_MISSED         = 0x000000010;
const s64 MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE      = 0x000000020;
const s64 MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE      = 0x000000040;
const s64 MOTOR_ERROR_MODULATION_MAGNITUDE            = 0x000000080;
const s64 MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION        = 0x000000100;
const s64 MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK       = 0x000000200;
const s64 MOTOR_ERROR_CURRENT_SENSE_SATURATION        = 0x000000400;
const s64 MOTOR_ERROR_CURRENT_LIMIT_VIOLATION         = 0x000001000;
const s64 MOTOR_ERROR_BRAKE_DUTY_CYCLE_NAN            = 0x000002000;
const s64 MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT       = 0x000004000;
const s64 MOTOR_ERROR_DC_BUS_OVER_CURRENT             = 0x000008000;
const s64 MOTOR_ERROR_MODULATION_IS_NAN               = 0x000010000;
const s64 MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP      = 0x000020000;
const s64 MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP        = 0x000040000;
const s64 MOTOR_ERROR_TIMER_UPDATE_MISSED             = 0x000080000;
const s64 MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE = 0x000100000;
const s64 MOTOR_ERROR_CONTROLLER_FAILED               = 0x000200000;
const s64 MOTOR_ERROR_I_BUS_OUT_OF_RANGE              = 0x000400000;
const s64 MOTOR_ERROR_BRAKE_RESISTOR_DISARMED         = 0x000800000;
const s64 MOTOR_ERROR_SYSTEM_LEVEL                    = 0x001000000;
const s64 MOTOR_ERROR_BAD_TIMING                      = 0x002000000;
const s64 MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE          = 0x004000000;
const s64 MOTOR_ERROR_UNKNOWN_PHASE_VEL               = 0x008000000;
const s64 MOTOR_ERROR_UNKNOWN_TORQUE                  = 0x010000000;
const s64 MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND         = 0x020000000;
const s64 MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT     = 0x040000000;
const s64 MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE            = 0x080000000;
const s64 MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND         = 0x100000000;
const s64 MOTOR_ERROR_UNKNOWN_GAINS                   = 0x200000000;
const s64 MOTOR_ERROR_CONTROLLER_INITIALIZING         = 0x400000000;
const s64 MOTOR_ERROR_UNBALANCED_PHASES               = 0x800000000;


const int ENCODER_MODE_INCREMENTAL                 = 0;
const int ENCODER_MODE_HALL                        = 1;
const int ENCODER_MODE_SINCOS                      = 2;
const int ENCODER_MODE_SPI_ABS_CUI                 = 256;
const int ENCODER_MODE_SPI_ABS_AMS                 = 257;
const int ENCODER_MODE_SPI_ABS_AEAT                = 258;
const int ENCODER_MODE_SPI_ABS_RLS                 = 259;
const int ENCODER_MODE_SPI_ABS_MA732               = 260;

const int CONTROL_MODE_VOLTAGE_CONTROL  = 0;
const int CONTROL_MODE_TORQUE_CONTROL   = 1;
const int CONTROL_MODE_VELOCITY_CONTROL = 2;
const int CONTROL_MODE_POSITION_CONTROL = 3;

const int INPUT_MODE_INACTIVE                      = 0;
const int INPUT_MODE_PASSTHROUGH                   = 1;
const int INPUT_MODE_VEL_RAMP                      = 2;
const int INPUT_MODE_POS_FILTER                    = 3;
const int INPUT_MODE_MIX_CHANNELS                  = 4;
const int INPUT_MODE_TRAP_TRAJ                     = 5;
const int INPUT_MODE_TORQUE_RAMP                   = 6;
const int INPUT_MODE_MIRROR                        = 7;
const int INPUT_MODE_TUNING                        = 8;

inline void print_axis_error(s64 error)
{
	if (error & AXIS_ERROR_INVALID_STATE              ) printf("AXIS_ERROR_INVALID_STATE "              );
	if (error & AXIS_ERROR_DC_BUS_UNDER_VOLTAGE       ) printf("AXIS_ERROR_DC_BUS_UNDER_VOLTAGE "       );
	if (error & AXIS_ERROR_DC_BUS_OVER_VOLTAGE        ) printf("AXIS_ERROR_DC_BUS_OVER_VOLTAGE "        );
	if (error & AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT) printf("AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT ");
	if (error & AXIS_ERROR_BRAKE_RESISTOR_DISARMED    ) printf("AXIS_ERROR_BRAKE_RESISTOR_DISARMED "    );
	if (error & AXIS_ERROR_MOTOR_DISARMED             ) printf("AXIS_ERROR_MOTOR_DISARMED "             );
	if (error & AXIS_ERROR_MOTOR_FAILED               ) printf("AXIS_ERROR_MOTOR_FAILED "               );
	if (error & AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED) printf("AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED ");
	if (error & AXIS_ERROR_ENCODER_FAILED             ) printf("AXIS_ERROR_ENCODER_FAILED "             );
	if (error & AXIS_ERROR_CONTROLLER_FAILED          ) printf("AXIS_ERROR_CONTROLLER_FAILED "          );
	if (error & AXIS_ERROR_POS_CTRL_DURING_SENSORLESS ) printf("AXIS_ERROR_POS_CTRL_DURING_SENSORLESS " );
	if (error & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED     ) printf("AXIS_ERROR_WATCHDOG_TIMER_EXPIRED "     );
	if (error & AXIS_ERROR_MIN_ENDSTOP_PRESSED        ) printf("AXIS_ERROR_MIN_ENDSTOP_PRESSED "        );
	if (error & AXIS_ERROR_MAX_ENDSTOP_PRESSED        ) printf("AXIS_ERROR_MAX_ENDSTOP_PRESSED "        );
	if (error & AXIS_ERROR_ESTOP_REQUESTED            ) printf("AXIS_ERROR_ESTOP_REQUESTED "            );
	if (error & AXIS_ERROR_HOMING_WITHOUT_ENDSTOP     ) printf("AXIS_ERROR_HOMING_WITHOUT_ENDSTOP "     );
	if (error & AXIS_ERROR_OVER_TEMP                  ) printf("AXIS_ERROR_OVER_TEMP "                  );
	if (error & AXIS_ERROR_UNKNOWN_POSITION           ) printf("AXIS_ERROR_UNKNOWN_POSITION "           );
}

inline void print_motor_error(s64 error)
{
	if (error & MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE  ) printf("MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE ");
	if (error & MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE  ) printf("MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE ");
	if (error & MOTOR_ERROR_ADC_FAILED                     ) printf("MOTOR_ERROR_ADC_FAILED ");
	if (error & MOTOR_ERROR_DRV_FAULT                      ) printf("MOTOR_ERROR_DRV_FAULT ");
	if (error & MOTOR_ERROR_CONTROL_DEADLINE_MISSED        ) printf("MOTOR_ERROR_CONTROL_DEADLINE_MISSED ");
	if (error & MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE     ) printf("MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE ");
	if (error & MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE     ) printf("MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE ");
	if (error & MOTOR_ERROR_MODULATION_MAGNITUDE           ) printf("MOTOR_ERROR_MODULATION_MAGNITUDE ");
	if (error & MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION       ) printf("MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION ");
	if (error & MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK      ) printf("MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK ");
	if (error & MOTOR_ERROR_CURRENT_SENSE_SATURATION       ) printf("MOTOR_ERROR_CURRENT_SENSE_SATURATION ");
	if (error & MOTOR_ERROR_CURRENT_LIMIT_VIOLATION        ) printf("MOTOR_ERROR_CURRENT_LIMIT_VIOLATION ");
	if (error & MOTOR_ERROR_BRAKE_DUTY_CYCLE_NAN           ) printf("MOTOR_ERROR_BRAKE_DUTY_CYCLE_NAN ");
	if (error & MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT      ) printf("MOTOR_ERROR_DC_BUS_OVER_REGEN_CURRENT ");
	if (error & MOTOR_ERROR_DC_BUS_OVER_CURRENT            ) printf("MOTOR_ERROR_DC_BUS_OVER_CURRENT ");
	if (error & MOTOR_ERROR_MODULATION_IS_NAN              ) printf("MOTOR_ERROR_MODULATION_IS_NAN ");
	if (error & MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP     ) printf("MOTOR_ERROR_MOTOR_THERMISTOR_OVER_TEMP ");
	if (error & MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP       ) printf("MOTOR_ERROR_FET_THERMISTOR_OVER_TEMP ");
	if (error & MOTOR_ERROR_TIMER_UPDATE_MISSED            ) printf("MOTOR_ERROR_TIMER_UPDATE_MISSED ");
	if (error & MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE) printf("MOTOR_ERROR_CURRENT_MEASUREMENT_UNAVAILABLE ");
	if (error & MOTOR_ERROR_CONTROLLER_FAILED              ) printf("MOTOR_ERROR_CONTROLLER_FAILED ");
	if (error & MOTOR_ERROR_I_BUS_OUT_OF_RANGE             ) printf("MOTOR_ERROR_I_BUS_OUT_OF_RANGE ");
	if (error & MOTOR_ERROR_BRAKE_RESISTOR_DISARMED        ) printf("MOTOR_ERROR_BRAKE_RESISTOR_DISARMED ");
	if (error & MOTOR_ERROR_SYSTEM_LEVEL                   ) printf("MOTOR_ERROR_SYSTEM_LEVEL ");
	if (error & MOTOR_ERROR_BAD_TIMING                     ) printf("MOTOR_ERROR_BAD_TIMING ");
	if (error & MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE         ) printf("MOTOR_ERROR_UNKNOWN_PHASE_ESTIMATE ");
	if (error & MOTOR_ERROR_UNKNOWN_PHASE_VEL              ) printf("MOTOR_ERROR_UNKNOWN_PHASE_VEL ");
	if (error & MOTOR_ERROR_UNKNOWN_TORQUE                 ) printf("MOTOR_ERROR_UNKNOWN_TORQUE ");
	if (error & MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND        ) printf("MOTOR_ERROR_UNKNOWN_CURRENT_COMMAND ");
	if (error & MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT    ) printf("MOTOR_ERROR_UNKNOWN_CURRENT_MEASUREMENT ");
	if (error & MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE           ) printf("MOTOR_ERROR_UNKNOWN_VBUS_VOLTAGE ");
	if (error & MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND        ) printf("MOTOR_ERROR_UNKNOWN_VOLTAGE_COMMAND ");
	if (error & MOTOR_ERROR_UNKNOWN_GAINS                  ) printf("MOTOR_ERROR_UNKNOWN_GAINS ");
	if (error & MOTOR_ERROR_CONTROLLER_INITIALIZING        ) printf("MOTOR_ERROR_CONTROLLER_INITIALIZING ");
	if (error & MOTOR_ERROR_UNBALANCED_PHASES              ) printf("MOTOR_ERROR_UNBALANCED_PHASES ");
}

inline void check_odrive_errors(Endpoint* root, int& num_errors)
{
	s64 error = 0;
	if (!root->odrive_fw_is_milana())
	{
		(*root)("error").get(error); if (error) { printf("odrive: error: 0x%llx\n", error); num_errors++; }
	}
	(*root)("can")("error").get(error); if (error) { printf("odrive: can error: 0x%llx\n", error); num_errors++; }
}

inline void check_axis_errors(Endpoint* axis, const char* axis_name, int& num_errors)
{
	s64 error = 0;
	(*axis)                        ("error").get(error); if (error) { printf("odrive: %s error: 0x%llx ", axis_name, error); print_axis_error(error); printf("\n"); num_errors++; }
	if (axis->odrive_fw_is_milana())
	{
		(*axis)("fet_thermistor")      ("error").get(error); if (error) { printf("odrive: %s fet_thermistor: 0x%llx\n", axis_name, error); num_errors++; }
		(*axis)("motor_thermistor")    ("error").get(error); if (error) { printf("odrive: %s motor_thermistor: 0x%llx\n", axis_name, error); num_errors++; }
	}
	(*axis)("motor")               ("error").get(error); if (error) { printf("odrive: %s motor: 0x%llx ", axis_name, error); print_motor_error(error); printf("\n"); num_errors++; }
	(*axis)("controller")          ("error").get(error); if (error) { printf("odrive: %s controller: 0x%llx\n", axis_name, error); num_errors++; }
	(*axis)("encoder")             ("error").get(error); if (error) { printf("odrive: %s encoder: 0x%llx\n", axis_name, error); num_errors++; }
	(*axis)("sensorless_estimator")("error").get(error); if (error) { printf("odrive: %s sensorless_estimator: 0x%llx\n", axis_name, error); num_errors++; }
}

inline void clear_odrive_errors(Endpoint* root)
{
	if (!root->odrive_fw_is_milana())
	{
		(*root)("error").set(0);
	}
	(*root)("can")("error").set(0);
}

inline void clear_axis_errors(Endpoint* axis)
{
	(*axis)                        ("error").set(0);
	if (axis->odrive_fw_is_milana())
	{
		(*axis)("fet_thermistor")      ("error").set(0);
		(*axis)("motor_thermistor")    ("error").set(0);
	}
	(*axis)("motor")               ("error").set(0);
	(*axis)("controller")          ("error").set(0);
	(*axis)("encoder")             ("error").set(0);
	(*axis)("sensorless_estimator")("error").set(0);
}
