// Battery voltage reading from the I2C A2D converter (ADS1115).
// Note: I have two batteries connected in series. The A2D converter is connected between the batteries
// (with an extra cable) and the ground. The voltage is reduced with a voltage divider.
// It measures the voltage of the battery that provides the ground (battery 1).
// The ODrive also has an A2D converter, and we use its measured voltage of both batteries combined
// to deduce the voltage of the second battery. This saves some extra wiring and resistors. But you could
// connect both batteries to the A2D converter if you don't like to depend on ODrive for this.
// There currently is no way to automatically cut off power, so we just display the battery power state
// in control_ui and also in the console on startup and hope the user notices a low power state in time.
// before the lipos take damage.
//
// We also retrieve the jetson temperature here.

#include "battery.h"
#include "main.h"

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <algorithm>
#include <assert.h>

const int battery_sensor_address = 0x4b;
#define I2C_ADDRESS "/dev/i2c-1"

static int battery_sensor_file = -1;

static bool get_jetson_temperature()
{
	float max_temp = -1000;
	for (int i = 0; i < 6; i++)
	{
		if (i == 4)
		{
			// This one always reports 100°C
			continue;
		}
		char buffer[128];
		sprintf(buffer, "/sys/devices/virtual/thermal/thermal_zone%d/temp", i);
		FILE* file = fopen(buffer, "r");
		if (!file)
		{
			printf("Jetson Temperature: Something went wrong\n");
			return false;
		}
        
		float temp = atoi(fgets(buffer, 7, file)) / 1000.0f;

		//printf("temp%d : %.2f\n", i, temp);

		max_temp = std::max(max_temp, temp);

		fclose(file);
	}
	md.jetson_temperature = max_temp;
	return true;
}

static bool do_analog_request()
{
	// We only read one value here, so we can select a slow data rate here.
	// We do the request at the end of the frame, and in the next frame we retrieve the
	// value and do the next request for the next frame.
	const u8 data_rate = 0b011; // 64SPS
	//const u8 data_rate = 0b111; // 860SPS

	const int input_no = 0;

	// These three bytes are written to the ADS1115 to set the config register and start a conversion 
	//writeBuf[0] = 1;			// This sets the pointer register so that the following two bytes write to the config register
	//writeBuf[1] = 0xC3;   	// This sets the 8 MSBs of the config register (bits 15-8) to 11000011
	//writeBuf[2] = 0x03;  		// This sets the 8 LSBs of the config register (bits 7-0) to 00000011
	// Write writeBuf to the ADS1115, the 3 specifies the number of bytes we are writing,
	// this begins a single conversion
	u8 buffer[3];
	buffer[0] = 1;
	buffer[1] = 0b11000011 | (input_no << 4);
	buffer[2] = 0b00000011 | (data_rate << 5);
	if (write(battery_sensor_file, buffer, 3) != 3)
		return false;

	return true;
}

static bool read_analog(float& value)
{
	u8 buffer[2];
	buffer[0] = 0;
	buffer[1] = 0;
	int count = 0;
	// Wait for the conversion to complete, this requires bit 15 to change from 0->1
	while (true)
	{
		if (read(battery_sensor_file, buffer, 2) != 2)
			return false;
		if ((buffer[0] & 0x80) != 0)
			break;
		if (++count == 100)
			break;
	}
	//if (count != 0) printf("analog count: %d\n", count);
	md.battery_sensor_counter = count;

	// Set pointer register to 0 to read from the conversion register
	buffer[0] = 0;
	if (write(battery_sensor_file, buffer, 1) != 1)
		return false;

	// Read the contents of the conversion register into readBuf
	if (read(battery_sensor_file, buffer, 2) != 2)
		return false;

	u16 val = (u16)((buffer[0] << 8) | buffer[1]); // Combine the two bytes of readBuf into a single 16 bit result 
	value = (float)val * 4.096f / 32767.0f;
	return true;
}


static int open_analog_sensor(int address)
{
	int file = open(I2C_ADDRESS, O_RDWR);
	if (file == -1)
		return -1;
	if (ioctl(file, I2C_SLAVE, address) < 0)
	{
		close(file);
		return -1;
	}
	return file;
}

bool battery_init()
{
	battery_sensor_file = open_analog_sensor(battery_sensor_address);
	if (battery_sensor_file == -1)
	{
		fprintf(stderr, "Battery: Failed to initialize analog sensor\n");
		return false;
	}

	if (!do_analog_request())
	{
		fprintf(stderr, "Battery: Failed to do analog request\n");
		return false;
	}
	precise_sleep(cd.target_delta_time_ms * 0.001);
	if (!battery_update())
	{
		fprintf(stderr, "Battery: Failed to do first update\n");
		return false;
	}
	printf("Battery1: %02i%%\n", int(md.battery_status_1 * 100.f));
	printf("Battery2: %02i%%\n", int(md.battery_status_2 * 100.f));
	return true;
}

void battery_close()
{
	if (battery_sensor_file != -1)
	{
		close(battery_sensor_file);
		battery_sensor_file = -1;
	}
}

static bool battery_update_helper()
{
	u32_micros start_time = time_micros();

	if (!read_analog(md.battery_1_sensor_voltage))
		return false;
	if (!do_analog_request())
		return false;

	md.battery_1_voltage = md.battery_1_sensor_voltage * 3.738f;

	// battery_total_voltage is measured in ODrive and retrieved in leg_control.cpp
	// It fluctuates a bit, so we smooth it also here.
	static float battery_total_voltage_smooth = 0;
	if (md.counter < 10) battery_total_voltage_smooth = md.battery_total_voltage;
	battery_total_voltage_smooth = battery_total_voltage_smooth * 0.98f + md.battery_total_voltage * 0.02f;
	
	float battery_2_voltage = battery_total_voltage_smooth + 0.046f - md.battery_1_voltage;

	// These is the safe voltage range for the lipos I use.
	const float max_voltage = 12.3f;
	const float min_voltage = 10.95f;
	md.battery_status_1 = (md.battery_1_voltage-min_voltage) / (max_voltage-min_voltage);
	md.battery_status_2 = (battery_2_voltage   -min_voltage) / (max_voltage-min_voltage);

	if (!get_jetson_temperature())
		return false;

	md.delta_time_battery = time_micros() - start_time;
	return true;
}

bool battery_update()
{
	bool r = battery_update_helper();
	if (!r)
	{
		perror("Battery");
	}
	return true;
}
