// We can control the robot with a wireless controller whose sensor is plugged into one
// of Jetsons USB ports. This code handles all that.
// With the controller, we can do all this:
// - Trigger startup sequence (start button)
// - Do emergency stop (Xbox button in center). This will turn off all motors instantly.
// - Spin left and right (Analog trigger buttons left and right). This is also used to turn left and right
//   when moving (it will turn slower then, so the robot won't fall down).
// - Move forwards and backwards (left analog stick, up and down)
// - By default, the maximum movement speed is very slow. You can speed up by holding A and slow down by holding X.
//   Releasing the left analog stick will always stop it though.
// - Move hip and leg joints with the right analog stick (all directions)
// - Lean left and right by holding the left digital trigger and moving the right analog stick left and right.
// - Jump by holding the left digital trigger button and press Y. The robot needs to be "kneeling" first to do this.
// - You can also control the arms. First you need to select "arm mode" by pressing the back button.
//   Then you can control where the arm endpoints are in space with the A,X,B,Y buttons. The leg and hip
//   will move automatically if it's required to reach the target point. You can also keep moving the
//   leg and hip joints, or move to the side, in arm mode. It will then try to keep the arm endpoints fixed
//   in space while moving the rest. To exit arm mode, press back again.
// 
// Of course you can change the mapping in this file, or add other things to control.

#include <stdio.h>
#include <sys/param.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <assert.h>
#include <libusb-1.0/libusb.h>

#include "main.h"
#include "tts_client.h"
#include "../common/geometry.h"

// Note: It's better to use the event file in /dev/input/by-id rather than the numbered ones
// in /dev/input because those change names depending on what usb devices are plugged in.
// You will probably have to change the name here, depending on what kind of controller you use.
const char* joystick_event_path = "/dev/input/by-id/usb-ShanWan_Wireless_Gamepad-event-joystick";
int file_joystick = -1;
pid_t xbox_driver_pid = 0;
libusb_context* ctx;

// Some controller need to have a certain driver running to work on Linux...
pid_t start_xbox_driver()
{
	printf("start xbox driver\n");
	pid_t xbox_driver_pid = fork();
	if (xbox_driver_pid == 0)
	{
		setpgid(getpid(), getpid());
		//printf("starting child %d: xbox_driver\n", getpid());
		int r = system("xboxdrv --detach-kernel-driver --silent --quiet");
		if (r < 0)
			printf("failed to start xboxdrv\n");
		exit(0);
	}
	if (xbox_driver_pid > 0)
		precise_sleep(0.1);
	return xbox_driver_pid;
}

bool is_usb_connected()
{
	const u16 VID = 0x045E;
	const u16 PID = 0x02EA;
	libusb_device_handle* handle = libusb_open_device_with_vid_pid(ctx, VID, PID);
	if (!handle)
	{
		return false;
	}
	libusb_close(handle);
	return true;
}

bool joystick_init()
{
	libusb_init(&ctx);

	if (is_usb_connected())
		xbox_driver_pid = start_xbox_driver();

	file_joystick = open(joystick_event_path, O_RDWR);
	if (file_joystick == -1)
	{
		printf("no controller found\n");
		return true;
	}
	int flags = fcntl(file_joystick, F_GETFL, 0);
	fcntl(file_joystick, F_SETFL, flags | O_NONBLOCK);
	return true;
}

const int button_a = 304;
const int button_x = 307;
const int button_b = 305;
const int button_y = 308;
const int button_menu_back = 314;
const int button_menu_start = 315;
const int button_menu_center = 316;
const int joystick_r_x = 2;
const int joystick_r_y = 5;
const int joystick_l_y = 1;
// const int joystick_l_x = defect;
const int trigger_l_analog = 10;
const int trigger_r_analog = 9;
const int trigger_l_digital = 310;
const int trigger_r_digital = 311;

const int num_buttons = 312;
bool pressing_button[num_buttons];
float trigger_r_analog_value = 0;
float trigger_l_analog_value = 0;

float arm_mode_vel_target = 0;

float value_to_signed_float(int value)
{
	if (value == 127 || value == 128) return 0;
	return (value / 255.0f - 0.5f) * 2;
}

// todo duplicated in Controller2
void do_startup()
{
	cd.startup_sequence_trigger++;
	cd.servo_enable = true;
	cd.enable_stand_control = true;
	cd.balance_control_enable = true;
}
void do_emergency_stop()
{
	// todo: duplicated in Controller2 GLFW_KEY_SPACE handling.
	printf("do_emergency_stop\n");
	cd.leg_control_enable_motor[0] = false;
	cd.leg_control_enable_motor[1] = false;
	cd.foot_control_enable_motor[0] = false;
	cd.foot_control_enable_motor[1] = false;
	cd.enable_stand_control = false;
	cd.balance_control_enable = false;
	cd.servo_enable = false;
	cd.servo_manual_enable = false;
	cd.arm_enable_angle_control = false;
	cd.stop_startup_sequence_trigger++;
	cd.counter++;
	cd.odrive_init_counter++;
}

void on_button(int code, int value)
{
	if (code >= 0 && code < num_buttons)
		pressing_button[code] = value != 0;


	switch (code)
	{
	//case button_a: cd.stand_control_vel = (value ?  cd.stand_control_vel_user_max : 0); break;
	//case button_x: cd.stand_control_vel = (value ? -cd.stand_control_vel_user_max : 0); break;
	//case button_b: cd.common_servo_target_vel_user = (value ?  cd.common_servo_vel_user_max : 0); break;
	//case button_y: cd.common_servo_target_vel_user = (value ? -cd.common_servo_vel_user_max : 0); break;
	case joystick_r_x:
		if (pressing_button[trigger_l_digital])
		{
			md.leaning_target_angle_joystick = -value_to_signed_float(value) * cd.leaning_max_angle;
			// hack: changing cd in robot code
			cd.common_leg_vel_target = 0;
			md.joystick_input_timer = 0;
		}
		else
		{
			// hack: changing cd in robot code
			cd.common_leg_vel_target = value_to_signed_float(value) * cd.common_leg_vel_user_max;
			md.leaning_target_angle_joystick = 0;
		}
		break;
	case joystick_r_y:
		if (pressing_button[trigger_l_digital])
		{
			// hack: changing cd in robot code
			cd.common_servo_target_vel_user = 0;
			md.joystick_input_timer = 0;
		}
		else
		{
			// hack: changing cd in robot code
			cd.common_servo_target_vel_user = value_to_signed_float(value) * cd.common_servo_vel_user_max;
			md.leaning_target_angle_joystick = 0;
		}
		break;

	case button_menu_back:
		if (value)
		{
			switch_arm_mode_smooth(md);
			tts_say_text(md.ik_enable_arm_mode ? "arm mode" : "normal mode");
			if (md.ik_enable_arm_mode)
			{
				// hack: changing cd in robot code
				cd.balance_control_vel_target = 0;
			}
			else
				arm_mode_vel_target = 0;
		}
		break;
	case button_menu_start : if (value) do_startup(); break;
	case button_menu_center: if (value) do_emergency_stop(); break;
	
	//case joystick_r_x: cd.balance_control_rotation_vel_user_target = -(value-127.0f) / 127.0f * cd.balance_control_rotation_vel_user_max; break;
	case joystick_l_y:
		if (md.ik_enable_arm_mode)
			arm_mode_vel_target  = -value_to_signed_float(value) * cd.balance_control_vel_user_max * 0.5f;
		else
		{
			// hack: changing cd in robot code
			cd.balance_control_vel_target  = -value_to_signed_float(value) * cd.balance_control_vel_user_max;
		}
		break;

	case trigger_r_analog: trigger_r_analog_value = value / 255.0f; break;
	case trigger_l_analog: trigger_l_analog_value = value / 255.0f; break;
	
	case button_y:
		if (!md.ik_enable_arm_mode && value && pressing_button[trigger_l_digital])
		{
			//printf("jump\n");
			// hack: changing cd in robot code
			cd.jump_phase_trigger++;
		}
		break;
	}

	if (code == trigger_r_analog || code == trigger_l_analog)
	{
		float dir = trigger_l_analog_value-trigger_r_analog_value;
		// hack: changing cd in robot code
		cd.balance_control_rotation_vel_user_target = dir * cd.balance_control_rotation_vel_user_max;
	}
}

bool joystick_update()
{
	u32_micros start_time = time_micros();

	md.joystick_input_timer += md.delta_time;

	if (md.ik_enable_arm_mode)
	{
		int dir = (int)pressing_button[button_a]-(int)pressing_button[button_x];
		md.ik_arm_target_z += dir * 50.0f * md.delta_time;
		md.ik_arm_target_z = clamp(md.ik_arm_target_z, -500, -15);

		dir = (int)pressing_button[button_b]-(int)pressing_button[button_y];
		md.ik_arm_target_y += dir * 50.0f * md.delta_time;
		md.ik_arm_target_y = clamp(md.ik_arm_target_y, 40, 150);

		if (pressing_button[trigger_l_digital])
		{
			md.ik_arm_target_x += arm_mode_vel_target * tau*wheel_radius*md.delta_time;
			md.ik_arm_target_x = clamp(md.ik_arm_target_x, 50, 300);
		}
		else
		{
			md.ik_x_delta_target += arm_mode_vel_target * tau*wheel_radius*md.delta_time;
			md.ik_x_delta_target = clamp(md.ik_x_delta_target, -100, 100);
		}
	}
	else
	{
		int dir = (int)pressing_button[button_a]-(int)pressing_button[button_x];
		md.balance_control_vel_extra_factor += dir * cd.balance_control_vel_extra_factor_speed*md.delta_time;
		//if (abs(cd.balance_control_vel_target) < cd.balance_control_vel_user_max * 0.5f)
		//	md.balance_control_vel_extra_factor -= cd.balance_control_vel_extra_factor_speed*md.delta_time*0.2f;
		md.balance_control_vel_extra_factor = clamp(md.balance_control_vel_extra_factor, 1, cd.balance_control_vel_extra_factor_max);
	}

	if (md.joystick_input_timer >= 5)
	{
		md.leaning_target_angle_joystick = move_towards(md.leaning_target_angle_joystick, 0, 0.1f*md.delta_time);
	}


	if (file_joystick == -1 && xbox_driver_pid == 0 && is_usb_connected())
	{
		xbox_driver_pid = start_xbox_driver();
	}
	if (file_joystick == -1/* && xbox_driver_pid != 0*/)
	{
		file_joystick = open(joystick_event_path, O_RDWR);
		if (file_joystick != -1)
		{
			printf("joystick reconnected\n");
			int flags = fcntl(file_joystick, F_GETFL, 0);
			fcntl(file_joystick, F_SETFL, flags | O_NONBLOCK);
		}
	}
	if (file_joystick != -1)
	{
		while (true)
		{
			// https://www.kernel.org/doc/Documentation/input/input.txt
			struct input_event
			{
				  struct timeval time;
				  unsigned short type;
				  unsigned short code;
				  unsigned int value;
			};
			input_event e;
			ssize_t n = read(file_joystick, (void*)&e, sizeof(e));
			//printf("read: %d\n", n);
			if (n != sizeof(e))
			{
				if (n == -1 && errno == EAGAIN)
					break;
				printf("joystick disconnected\n");
				file_joystick = -1;
				break;
			}
			on_button(e.code, e.value);
			static bool not_first = false;
			if (e.code && !not_first)
			{
				printf("%d %d %d\n", e.type, e.code, e.value);
				not_first = true;
			}
		}
	}
	md.delta_time_joystick = time_micros() - start_time;
	return true;
}

void joystick_close()
{
	if (file_joystick != -1)
		close(file_joystick);
	file_joystick = -1;

	if (xbox_driver_pid > 0)
	{
	    //printf("kill xbox_driver_pid %d\n", xbox_driver_pid);
	    kill(-xbox_driver_pid, SIGKILL);
    }
	xbox_driver_pid = 0;

	libusb_exit(ctx);
}
