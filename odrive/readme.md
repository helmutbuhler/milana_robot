# ODrive
## Overview
This robot uses two ODrives to control the 4 BLDC Motors that control the feet and leg motors. Some modifications were required to the Firmware, which you can find [here](todo). Clone that repository into the user home folder on the Jetson and flash the Firmware onto the ODrives (instructions are there).

The ODrive connected to the leg motors is located in the front of the robot (opposite of batteries). To connect to it with ODrivetool via UART, use this:
```
python3 ~/odrive_milana/tools/odrivetool --path serial:/dev/ttyTHS1_921600_8N2
```
Its axis numbers are: left=1 right=0

The ODrive connected to the foot motors is located in the back of the robot (same side as batteries). To connect to it with ODrivetool via UART, use this:
```
python3 ~/odrive_milana/tools/odrivetool --path serial:/dev/ttyS0_921600_8N2
```
Its axis numbers are: left=0 right=1

To connect to an ODrive via USB, use this:
```
python3 ~/odrive_milana/tools/odrivetool
```
After flashing the ODrive, you first need to configure them via USB before you can access them with UART. In the following are the commands to initialize them with ODriveTool:


## Leg ODrive Initialization
Note: The requested_state assignment in the bottom will start motor calibration. Make sure the motors are free to rotate for that.
```
odrv0.erase_configuration()
odrv0.config.uart_baudrate = 921600
odrv0.config.uart_stop_bits = 2
odrv0.config.brake_resistance = 0
odrv0.config.max_regen_current = 0
odrv0.config.dc_max_negative_current = -15

odrv0.axis0.motor.config.current_lim = 10
odrv0.axis0.motor.config.pole_pairs = 20
odrv0.axis0.motor.config.torque_constant = 1
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.motor.config.requested_current_range = 120
odrv0.axis0.encoder.config.abs_spi_cs_gpio_pin = 4
odrv0.axis0.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS #257
odrv0.axis0.encoder.config.cpr = 2**14
odrv0.axis0.encoder.config.use_index = False
odrv0.axis0.encoder.config.ignore_abs_ams_error_flag = True

odrv0.axis1.motor.config.current_lim = 10
odrv0.axis1.motor.config.pole_pairs = 20
odrv0.axis1.motor.config.torque_constant = 1
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.requested_current_range = 120
odrv0.axis1.encoder.config.abs_spi_cs_gpio_pin = 5
odrv0.axis1.encoder.config.mode = ENCODER_MODE_SPI_ABS_AMS #257
odrv0.axis1.encoder.config.cpr = 2**14
odrv0.axis1.encoder.config.use_index = False
odrv0.axis1.encoder.config.ignore_abs_ams_error_flag = True

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True

odrv0.save_configuration()
```

## Foot ODrive Initialization
Note: The requested_state assignment in the bottom will start motor calibration. Make sure the motors are free to rotate for that.
```
odrv0.erase_configuration()
odrv0.config.uart_baudrate = 921600
odrv0.config.uart_stop_bits = 2
odrv0.config.brake_resistance = 0
odrv0.config.max_regen_current = 0
odrv0.config.dc_max_negative_current = -15

odrv0.axis0.motor.config.current_lim = 10
odrv0.axis0.motor.config.pole_pairs = 7
odrv0.axis0.motor.config.torque_constant = 1
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.motor.config.requested_current_range = 120
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL #0
odrv0.axis0.encoder.config.cpr = 8192
odrv0.axis0.encoder.config.use_index = True
odrv0.axis0.config.calibration_lockin.vel = -40
odrv0.axis0.config.calibration_lockin.ramp_distance = -3.141592653589
odrv0.axis0.config.calibration_lockin.accel = -20

odrv0.axis1.motor.config.current_lim = 10
odrv0.axis1.motor.config.pole_pairs = 7
odrv0.axis1.motor.config.torque_constant = 1
odrv0.axis1.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.requested_current_range = 120
odrv0.axis1.encoder.config.mode = ENCODER_MODE_INCREMENTAL #0
odrv0.axis1.encoder.config.cpr = 8192
odrv0.axis1.encoder.config.use_index = True
odrv0.axis1.config.calibration_lockin.vel = 40
odrv0.axis1.config.calibration_lockin.ramp_distance = 3.141592653589
odrv0.axis1.config.calibration_lockin.accel = 20

odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.encoder.config.pre_calibrated = True
odrv0.axis1.encoder.config.pre_calibrated = True

odrv0.save_configuration()
```

## Anticogging
To make the movement of the foot motors more smooth, Anticogging should be calibrated and enabled. The motors will work without this, but the balancing will work much better with Antocogging enabled. To learn more about Anticogging on ODrive, watch [this](https://www.youtube.com/watch?v=CWzuBoYLISY). Anticogging on the Leg motors could be theoretically enabled also, but because it has a 3 to 1 gear ratio you don't really notice the cogging there.

Use the following to run the calibration. The wheels must be free to rotate for this to work. I usually make the robot lay on its batteries and put the feet up in the air.
```
def start_anticogging_calibration(axis, vel_gain):
    axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    axis.controller.config.pos_gain = 100
    axis.controller.config.vel_gain = vel_gain
    axis.controller.config.vel_integrator_gain = 100
    axis.controller.config.anticogging.calib_pos_threshold = 1
    axis.controller.config.anticogging.calib_vel_threshold = 1
    axis.controller.config.anticogging.anticogging_enabled = False
    axis.motor.config.current_lim = 10
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    axis.controller.input_pos = 0
    print("init...")
    import time
    time.sleep(1)
    print("start_anticogging_calibration")
    axis.controller.start_anticogging_calibration()

start_anticogging_calibration(odrv0.axis0, 4)
start_anticogging_calibration(odrv0.axis1, 4)
```

You may need to tune the gains a little bit. Once both feet are calibrated, put them back into idle:
```
odrv0.axis0.requested_state = AXIS_STATE_IDLE
odrv0.axis1.requested_state = AXIS_STATE_IDLE
```

And save the calibration:
```
odrv0.axis1.controller.config.anticogging

odrv0.axis0.controller.config.anticogging.anticogging_enabled = True
odrv0.axis1.controller.config.anticogging.anticogging_enabled = True
odrv0.axis0.controller.config.anticogging.pre_calibrated = True
odrv0.axis1.controller.config.anticogging.pre_calibrated = True
odrv0.save_configuration()
```

You can optionally retrieve the calibrated values with the following. This is useful to debug encoder slippage.
```
def get_anticogging_map(axis, filename):
    with open(filename, 'w') as f:
        for i in range(3600):
            if i%100 == 0:
                print(i//100)
            val = axis.controller.get_anticogging_map(i)
            f.write(str(val))
            f.write('\n')
get_anticogging_map(odrv0.axis0, 'anticogging_map_0.csv')
get_anticogging_map(odrv0.axis1, 'anticogging_map_1.csv')
```

To check if the Anticogging is working, you can enable a constant slow speed in `control_ui` and plot the position. You can also enable/disable Anticogging there with a checkbox.
This Anticogging Calibration need to be done only once because the feet have absolute encoders. If you notice that it's not working anymore after some time, or after a jump, you likely have encoder slippage.
