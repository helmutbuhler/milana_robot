// Main program that is run on the Jetson Nano inside the robot.

#ifdef COMPILING_CONTROL_UI
#error COMPILING_CONTROL_UI must not be defined for the robot project
#endif

#include <cstdlib>
#include <stdio.h>
#include <sys/param.h>
#include <sys/resource.h>
#include <time.h>
#include <algorithm>
#include "main.h"
#include "server.h"
#include "leg_control.h"
#include "foot_control.h"
#include "battery.h"
#include "servo.h"
#include "balance_control.h"
#include "joystick.h"
#include "imu/spi_imu.h"
#include "body_acc.h"
#include "../common/leaning.h"
#include "../common/time_helper.h"
#include "ik.h"
#include "statistics.h"
#include "tts_client.h"
#include "asr_client.h"
#include "command.h"

bool running = true;
MonitorData md;
ControlData cd;

static void ctrl_c_handler(int signum)
{
	running = false;
}

static void setup_handlers()
{
	struct sigaction sa = { 0 };
	sa.sa_handler = ctrl_c_handler;

	// If we don't set SA_RESTART, the recv call to the uart file handle in ODrive will fail.
	sa.sa_flags = SA_RESTART | SA_NOCLDSTOP;

	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
}

bool init_i2c()
{
	FILE* file = fopen("/sys/bus/i2c/devices/i2c-1/bus_clk_rate", "w");
	if (!file)
	{
		perror("init_i2c");
		return false;
	}
    fprintf(file, "400000\n");
	fclose(file);
	return true;
}

int main(int argc, char *argv[])
{
	int result = EXIT_FAILURE;
	u32_micros last_time;
	setup_handlers();

	// Set higher priority to own process.
    if (setpriority(PRIO_PROCESS, 0, -20) != 0)
		printf("cannot set priority\n");
	
	//u64 start_time2 = time_micros_64();
	
	if (!joystick_init())       goto fail;
	if (!init_i2c())            goto fail;
	if (!leg_control_init())
	{
		battery_init();
		goto fail;
	}
	if (!battery_init())        goto fail;
	if (!tts_client_init())     goto fail;
	if (!foot_control_init())   goto fail;
	leg_control_init2();
	if (!balance_control_init())goto fail;
	if (!servo_motors_init())   goto fail;
	if (!body_acc_init())       goto fail;
	if (!leaning_init())        goto fail;
	if (!ik_init())             goto fail;
	if (!spi_imu_init())        goto fail;
	if (!server_init())         goto fail;
	if (!asr_client_init())     goto fail;
	if (!command_init())        goto fail;
	
	last_time = time_micros();
	md.delta_time = 0.004f;
	
	while (running)
	{
		if (!joystick_update())       goto fail;
		if (!battery_update())        goto fail;
		if (!body_acc_update())       goto fail;
		if (!leaning_update())        goto fail;
		if (!ik_update())             goto fail;
		if (!spi_imu_update())        goto fail;
		if (!leg_control_update())    goto fail;
		if (!foot_control_update_1()) goto fail;
		if (!balance_control_update())goto fail;
		if (!foot_control_update_2()) goto fail;
		if (!servo_motors_update())   goto fail;
		if (!statistics_update())     goto fail;
		if (!server_update())         goto fail;
		if (!tts_client_update())     goto fail;
		if (!asr_client_update())     goto fail;
		if (!command_update())        goto fail;
		
		// calculate delta time

		u32_micros start_time = time_micros();
		if (cd.target_delta_time_ms == 0)
			precise_sleep(0.001);
		else
		{
			double target_delta = cd.target_delta_time_ms * 0.001;
			double delta = double(start_time - last_time) * .000001;
			double remaining = target_delta - delta;
			precise_sleep(remaining);
		}

		static int last_pause_trigger;
		if (cd.pause_trigger-last_pause_trigger == 1)
			precise_sleep(cd.pause_delta_time_ms * 0.001);
		last_pause_trigger = cd.pause_trigger;

		if (cd.do_random_pauses)
			precise_sleep((rand()%cd.random_pauses_max_ms) * 0.001);

		u32_micros current_time = time_micros();
		md.delta_time_sleep = current_time - start_time;

		md.delta_time_micros = current_time - last_time;
		last_time = current_time;
		md.delta_time = float(md.delta_time_micros) * .000001f;

		md.uptime_jetson_micros = time_micros_64();
		time(&md.local_time);

		md.counter++;
	}
	printf("\n");
	result = EXIT_SUCCESS;
fail:
	//if (result == EXIT_FAILURE)
	//	tts_say_text("Something failed. Shutting down.");
	server_close();
	spi_imu_close();
	body_acc_close();
	leaning_close();
	ik_close();
	servo_motors_close();
	balance_control_close();
	battery_close();
	foot_control_close();
	leg_control_close();
	joystick_close();
	tts_client_close();
	asr_client_close();
	command_close();
	return result;
}
