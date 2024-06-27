// Retrieval of imu sensor data (gyroscope, accelerometer, temperature).
// To make it as precise as possible, we query the gyro at a very high frequency (faster than our mainloop).
// That's possible because we use the IMU-internal fifo buffer. We also use an thread to retrieve all this
// data, to speed up our mainloop.
// The sensor fusion of this data is done in sensor_fusion.cpp

#include "spi_imu.h"
#include "../../common/helper.h"
#include "../main.h"
#include "LSM6DS3.h"
#include "sensor_fusion.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>

#define SPI_PATH "/dev/spidev0.0"

// This is only 1M and not 10M, even though the chip documentation says it can handle 10M.
// The data gets unreliable otherwise. Don't know why, but this is fast enough.
const u32 speed = 1000000;
static int fd = -1;

const bool imu_use_fifo = true;

void gyro_thread_lock();
void gyro_thread_unlock();
bool start_gyro_thread();
void stop_gyro_thread();
void load_gyro_thread_values();

void transfer(u8* send, u8* receive, int length)
{
	spi_ioc_transfer t = {0};
	t.tx_buf = (u64)send;
	t.rx_buf = (u64)receive;
	t.len = length;
	t.speed_hz = speed;
	t.bits_per_word = 8;

	int status = ioctl(fd, SPI_IOC_MESSAGE(1), &t);
	if (status < 0)
	{
		perror("SPI: SPI_IOC_MESSAGE Failed");
		spi_imu_close();
	}
}

u8 read_register(u8 address)
{
	u8 data[2] = {(u8)(address | 0x80), 0};
	transfer(data, data, 2);
	return data[1];
}

s16 read_register_s16(u8 address)
{
	u8 data[3] = {(u8)(address | 0x80), 0, 0};
	transfer(data, data, 3);
	return (s16)data[1] + (s16)(data[2]<<8);
}

void write_register(u8 address, u8 value)
{
	u8 data[2] = {address, value};
	transfer(data, data, 2);
}

const float accel_range = 4;
const u8 accel_range_flags = LSM6DS3_ACC_GYRO_FS_XL_4g;
float convert_accel(s16 value)
{
	return (float)value * 0.061f * accel_range / 2000.0f;
}

const float gyro_range = 2000;
const u8 gyro_range_flags = LSM6DS3_ACC_GYRO_FS_G_2000dps;

float convert_gyro(s16 value)
{
	return (float)value * 4.375f * gyro_range / (125*1000);
}

bool spi_imu_init()
{
	fd = open(SPI_PATH, O_RDWR);
	if (fd < 0)
	{
		printf("SPI Error: Can't find IMU on: %s\n", SPI_PATH);
		perror("");
		return false;
	}
	u8 mode = 0;
	if (ioctl(fd, SPI_IOC_WR_MODE, &mode) == -1)
	{
		perror("SPI: Can't set SPI mode.");
		return false;
	}
	u8 bits = 8;
	if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
	{
		perror("SPI: Can't set bits per word.");
		return false;
	}
	if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1)
	{
		perror("SPI: Can't set max speed HZ");
		return false;
	}

	u8 who_am_i = read_register(LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
    if (who_am_i != LSM6DS3_ACC_GYRO_WHO_AM_I)
	{
		printf("SPI: Who am I: %x. Expected: %x\n", who_am_i, LSM6DS3_ACC_GYRO_WHO_AM_I);
		// Sometimes this happens, but it still seems to work.
		//return false;
    }

	write_register(LSM6DS3_ACC_GYRO_CTRL1_XL,
			LSM6DS3_ACC_GYRO_BW_XL_50Hz |
			LSM6DS3_ACC_GYRO_ODR_XL_52Hz |
			accel_range_flags);

	// The gyro frequency needs to be quite high, compared to the robot frequency.
	// If it is set to 50Hz (similar to robot frequency), there is a significant delay
	// in the retrieved data, which would cause the balance control to oscillate the robot quite heavily.
	switch (cd.imu_gyro_frequency)
	{
	case 0: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_13Hz  | gyro_range_flags); break;
	case 1: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_26Hz  | gyro_range_flags); break;
	case 2: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_52Hz  | gyro_range_flags); break;
	case 3: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_104Hz | gyro_range_flags); break;
	case 4: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_208Hz | gyro_range_flags); break;
	case 5: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_416Hz | gyro_range_flags); break;
	case 6: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_833Hz | gyro_range_flags); break;
	case 7: write_register(LSM6DS3_ACC_GYRO_CTRL2_G, LSM6DS3_ACC_GYRO_ODR_G_1660Hz| gyro_range_flags); break;
	}
	
	write_register(LSM6DS3_ACC_GYRO_CTRL4_C, LSM6DS3_ACC_GYRO_I2C_DISABLE_SPI_ONLY);

	if (imu_use_fifo)
	{
		// We use the internal fifo buffer to fetch the gyro data. This has the advantage that when we,
		// for some reason, have a very long frame, we can still get the full data.
		// This is especially important for jumps, where we have to do a lot of communication with
		// ODrive and where we cannot use the accelerometer.
		// This hopefully fixes imu deviations like in monitor_log_jump40.bin at 931 seconds.

		u16 fifoThreshold = 100;  //Can be 0 to 4096 (16 bit bytes) actual value doesn't matter here.
		u8 thresholdLByte = fifoThreshold & 0x00FF;
		u8 thresholdHByte = (fifoThreshold & 0x0F00) >> 8;
		write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL1, thresholdLByte);
		write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL2, thresholdHByte);

		int gyroFifoDecimation = 1; // no decimation
		u8 tempFIFO_CTRL3 = (gyroFifoDecimation & 0x07) << 3;
		write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL3, tempFIFO_CTRL3);

		// Disable fifo mode first. This clears the internal buffer.
		write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0);
		
		// Enable FIFO mode. Can be:
		//int fifoMode = 0; // (Bypass mode, FIFO off)
		//int fifoMode = 1; // (Stop when full)
		//int fifoMode = 3; // (Continuous during trigger)
		//int fifoMode = 4; // (Bypass until trigger)
		int fifoMode = 6; // (Continous mode)
		switch (cd.imu_fifo_frequency)
		{
		case 0: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_10Hz    | fifoMode); break;
		case 1: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_25Hz    | fifoMode); break;
		case 2: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_50Hz    | fifoMode); break;
		case 3: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_100Hz   | fifoMode); break;
		case 4: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_200Hz   | fifoMode); break;
		case 5: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_400Hz   | fifoMode); break;
		case 6: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_800Hz   | fifoMode); break;
		case 7: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_1600Hz  | fifoMode); break;
		case 8: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_3300Hz  | fifoMode); break;
		case 9: write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_6600Hz  | fifoMode); break;
		case 10:write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, LSM6DS3_ACC_GYRO_ODR_FIFO_13300Hz | fifoMode); break;
		}
	}
	else
	{
		write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL3, 0);
		write_register(LSM6DS3_ACC_GYRO_FIFO_CTRL5, 0);
	}

	// do test reading
	int counter = 0;
	while (true)
	{
		if (read_register_s16(LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0)
			break;
		counter++;
	}
	//if (counter)
	//	printf("SPI: counter: %d\n", counter);
	if (fd == -1)
		return false;

	if (!start_gyro_thread())
		return false;

	return true;
}

bool spi_imu_update()
{
	u32_micros start_time = time_micros();
	
	gyro_thread_lock();
	if (fd == -1)
	{
		gyro_thread_unlock();
		return false;
	}

	/*bool reset_imu = false;
	static int set_imu_gyro_frequency = -1;
	if (set_imu_gyro_frequency != cd.imu_gyro_frequency)
	{
		reset_imu = true;
		set_imu_gyro_frequency = cd.imu_gyro_frequency;
	}

	static int set_imu_fifo_frequency = -1;
	if (set_imu_fifo_frequency != cd.imu_fifo_frequency)
	{
		reset_imu = true;
		set_imu_fifo_frequency = cd.imu_fifo_frequency;
	}
	if (reset_imu)
	{
		spi_imu_close();
		if (!spi_imu_init())
		{
			gyro_thread_unlock();
			return false;
		}
	}*/

	memset(md.imu_fifo_gyro_x, 0, sizeof(md.imu_fifo_gyro_x)*3);
	if (imu_use_fifo)
	{
		md.imu_fifo_gyro_count = 0;
		if (cd.imu_use_gyro_thread)
			load_gyro_thread_values();

		while ((read_register_s16(LSM6DS3_ACC_GYRO_FIFO_STATUS1) & 0x1000) == 0 && running)
		{
			md.imu_gyro_x = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L));
			md.imu_gyro_y = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L));
			md.imu_gyro_z = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L));
			if (md.imu_fifo_gyro_count < imu_fifo_gyro_max_count)
			{
				md.imu_fifo_gyro_x[md.imu_fifo_gyro_count] = md.imu_gyro_x;
				md.imu_fifo_gyro_y[md.imu_fifo_gyro_count] = md.imu_gyro_y;
				md.imu_fifo_gyro_z[md.imu_fifo_gyro_count] = md.imu_gyro_z;
			}
			md.imu_fifo_gyro_count++;
			do_sensor_fusion_gyro(md, true);
		}
	}
	else
	{
		md.imu_gyro_x = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_OUTX_L_G));
		md.imu_gyro_y = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_OUTY_L_G));
		md.imu_gyro_z = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_OUTZ_L_G));
		md.imu_fifo_gyro_count = -1;
		do_sensor_fusion_gyro(md, false);
	}

	md.imu_gravity_acc_x = convert_accel(read_register_s16(LSM6DS3_ACC_GYRO_OUTX_L_XL));
	md.imu_gravity_acc_y = convert_accel(read_register_s16(LSM6DS3_ACC_GYRO_OUTY_L_XL));
	md.imu_gravity_acc_z = convert_accel(read_register_s16(LSM6DS3_ACC_GYRO_OUTZ_L_XL));
	
	md.imu_temperature = (float)read_register_s16(LSM6DS3_ACC_GYRO_OUT_TEMP_L) / 16 + 25;

	// Calculate temperature bias for gyroscope
	// For some reason, the temperature measured by odrive is better correlated with the z-axis gyro.
	md.imu_temp_gyro_x = md.imu_temperature    *cd.imu_temp_to_gyro_x_a+cd.imu_temp_to_gyro_x_b;
	md.imu_temp_gyro_y = md.imu_temperature    *cd.imu_temp_to_gyro_y_a+cd.imu_temp_to_gyro_y_b;
	md.imu_temp_gyro_z = md.odrive_temperature *cd.imu_temp_to_gyro_z_a+cd.imu_temp_to_gyro_z_b;
	
	gyro_thread_unlock();

	do_sensor_fusion_acc();

	md.delta_time_imu = time_micros() - start_time;
	return true;
}

void spi_imu_close()
{
	gyro_thread_lock();
	if (fd != -1)
	{
		// Disable sensor reading to save some power while this application isn't running.
		write_register(LSM6DS3_ACC_GYRO_CTRL2_G,  LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN);
		write_register(LSM6DS3_ACC_GYRO_CTRL1_XL, LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN);
		close(fd);
	}
	fd = -1;
	gyro_thread_unlock();
	stop_gyro_thread();
}


// Retrieval of fifo gyro data is relatively slow, so we make an extra thread which extracts all the data
// during the entire main frame time.
// This thread temporarilly stores the data in these global variables, until the main thread retrieves it
// with load_gyro_thread_values().
// To synchronize access to the imu chip, a mutex is used.
// Without this thread, retrieval time is at least 6ms. With it, the main thread takes at most 1.2ms.
static pthread_t thread_id;
static bool thread_running;
static pthread_mutex_t lock;

static float thread_imu_fifo_gyro_x[imu_fifo_gyro_max_count] = {0};
static float thread_imu_fifo_gyro_y[imu_fifo_gyro_max_count] = {0};
static float thread_imu_fifo_gyro_z[imu_fifo_gyro_max_count] = {0};
static int thread_imu_fifo_gyro_count = 0;

void gyro_thread_lock()
{
    pthread_mutex_lock(&lock);
}

void gyro_thread_unlock()
{
    pthread_mutex_unlock(&lock);
}

void* gyro_thread(void*)
{
	while (true)
	{
		gyro_thread_lock();
		while (fd != -1 && running &&
				thread_imu_fifo_gyro_count < imu_fifo_gyro_max_count &&
				(read_register_s16(LSM6DS3_ACC_GYRO_FIFO_STATUS1) & 0x1000) == 0)
		{
			float imu_gyro_x = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L));
			float imu_gyro_y = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L));
			float imu_gyro_z = convert_gyro(read_register_s16(LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L));
			thread_imu_fifo_gyro_x[thread_imu_fifo_gyro_count] = imu_gyro_x;
			thread_imu_fifo_gyro_y[thread_imu_fifo_gyro_count] = imu_gyro_y;
			thread_imu_fifo_gyro_z[thread_imu_fifo_gyro_count] = imu_gyro_z;
			thread_imu_fifo_gyro_count++;
		}
		if (fd == -1)
		{
			// Setting fd to -1 signals this thread to stop.
			gyro_thread_unlock();
			break;
		}
		gyro_thread_unlock();

		precise_sleep(0.001);
	}
	return nullptr;
}

bool start_gyro_thread()
{
	assert(!thread_running);

	pthread_mutexattr_t ma;
    pthread_mutexattr_init(&ma);
    pthread_mutexattr_settype(&ma, PTHREAD_MUTEX_RECURSIVE);
	if (pthread_mutex_init(&lock, &ma) != 0)
        return false;

	int r = pthread_create(&thread_id, nullptr, gyro_thread, nullptr);
	if (r != 0)
		return false;

	thread_running = true;
	thread_imu_fifo_gyro_count = 0;
	return true;
}

void stop_gyro_thread()
{
	assert(fd == -1);
	if (!thread_running)
		return;
	pthread_join(thread_id, nullptr);
    pthread_mutex_destroy(&lock);
	thread_running = false;
}

void load_gyro_thread_values()
{
	// This is called from the main thread
	// The mutex is assumed to be locked when calling this.
	for (int i = 0; i < thread_imu_fifo_gyro_count; i++)
	{
		md.imu_gyro_x = md.imu_fifo_gyro_x[i] = thread_imu_fifo_gyro_x[i];
		md.imu_gyro_y = md.imu_fifo_gyro_y[i] = thread_imu_fifo_gyro_y[i];
		md.imu_gyro_z = md.imu_fifo_gyro_z[i] = thread_imu_fifo_gyro_z[i];
		do_sensor_fusion_gyro(md, true);
	}
	md.imu_fifo_gyro_count = thread_imu_fifo_gyro_count;
	thread_imu_fifo_gyro_count = 0;
}
