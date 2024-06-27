#include "pin.h"
#include <stdio.h>
#include <algorithm>
//#include <pigpio.h>
#include <assert.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

int exported_pins[64];
int num_exported_pins = 0;

#define SYSFS_GPIO_DIR "/sys/class/gpio"
//#define POLL_TIMEOUT (3 * 1000) /* 3 seconds */
#define MAX_BUF 64

#ifndef _MSC_VER
#pragma GCC diagnostic ignored "-Wunused-result"
#endif


// source: JETSON_NANO_PIN_DEFS in gpio_pin_data.py in https://github.com/NVIDIA/jetson-gpio.git
struct JetsonMapping
{
	int jetson;
	int pin;
	int pi;
};
JetsonMapping jetson_mapping[] = {
    {216, 7 , 4 },
    {50 , 11, 17},
    {79 , 12, 18},
    {14 , 13, 27},
    {194, 15, 22},
    {232, 16, 23},
    {15 , 18, 24},
    {16 , 19, 10},
    {17 , 21, 9 },
    {13 , 22, 25},
    {18 , 23, 11},
    {19 , 24, 8 },
    {20 , 26, 7 },
    {149, 29, 5 },
    {200, 31, 6 },
    {168, 32, 12},
    {38 , 33, 13},
    {76 , 35, 19},
    {51 , 36, 16},
    {12 , 37, 26},
    {77 , 38, 20},
    {78 , 40, 21},
};
int pi_to_jetson[30];


bool pin_init()
{
	//if (gpioInitialise()<0) return 1;
	for (int i = 0; i < sizeof(jetson_mapping) / sizeof(jetson_mapping[0]); i++)
	{
		pi_to_jetson[jetson_mapping[i].pi] = jetson_mapping[i].jetson;
	}
	return true;
}

int pin_unexport(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];
 
	//printf("unexport %d\n", gpio);

	fd = open(SYSFS_GPIO_DIR "/unexport", O_WRONLY);
	if (fd < 0) {
		printf("unexport %d failed!\n", gpio);
		//perror("gpio/export");
		return fd;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	int r = write(fd, buf, len);
	//assert(r == len);
	close(fd);
	return 0;
}

void pin_close()
{
	//gpioTerminate();
	/*for (int i = 0; i < num_exported_pins; i++)
		pin_unexport(exported_pins[i]);
	num_exported_pins = 0;*/
}




 /****************************************************************
 * Constants
 ****************************************************************/
 

bool pin_export(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	//printf("export %d\n", gpio);

	fd = open(SYSFS_GPIO_DIR "/export", O_WRONLY);
	if (fd < 0) {
		perror("gpio/export");
		return false;
	}
 
	len = snprintf(buf, sizeof(buf), "%d", gpio);
	int r = write(fd, buf, len);
	if (r < 0)
		printf("err %s\n", strerror(errno));
	assert(r == len);
	close(fd);
	if (r >= 0)
	{
		if (num_exported_pins < 64)
			exported_pins[num_exported_pins++] = gpio;
		else
			assert(0);
	}
	return r >= 0;
}


/****************************************************************
 * gpio_set_dir
 ****************************************************************/
bool pin_set_mode(unsigned int pin, PinMode mode)
{
	int fd, len;
	char buf[MAX_BUF];

	int gpio = pi_to_jetson[pin];
	assert(gpio);
	pin_unexport(gpio);
	pin_export(gpio);

	//printf("set_mode %d\n", gpio);
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR  "/gpio%d/direction", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0)
	{
		perror("pin_set_mode");
		return false;
	}
 
	if (mode == pin_mode_out)
	{
		int r = write(fd, "out", 4);
		assert(r == 4);
	}
	else
	{
		int r = write(fd, "in", 3);
		assert(r == 3);
	}
 
	close(fd);
	return true;
}

/****************************************************************
 * gpio_set_value
 ****************************************************************/
bool pin_set(unsigned int pin, unsigned int value)
{
	int fd, len;
	char buf[MAX_BUF];

	int gpio = pi_to_jetson[pin];
 
	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-value");
		return false;
	}
 
	int r = write(fd, value ? "1" : "0", 2);
	if (r < 0)
		printf("pin_set %s\n", strerror(errno));
 
	close(fd);
	return true;
}

/****************************************************************
 * gpio_get_value
 ****************************************************************/
int gpio_get_value(unsigned int gpio, unsigned int *value)
{
	int fd, len;
	char buf[MAX_BUF];
	char ch;

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY);
	if (fd < 0) {
		perror("gpio/get-value");
		return fd;
	}
 
	read(fd, &ch, 1);

	if (ch != '0') {
		*value = 1;
	} else {
		*value = 0;
	}
 
	close(fd);
	return 0;
}


/****************************************************************
 * gpio_set_edge
 ****************************************************************/

int gpio_set_edge(unsigned int gpio, char *edge)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/edge", gpio);
 
	fd = open(buf, O_WRONLY);
	if (fd < 0) {
		perror("gpio/set-edge");
		return fd;
	}
 
	write(fd, edge, strlen(edge) + 1); 
	close(fd);
	return 0;
}

/****************************************************************
 * gpio_fd_open
 ****************************************************************/

int gpio_fd_open(unsigned int gpio)
{
	int fd, len;
	char buf[MAX_BUF];

	len = snprintf(buf, sizeof(buf), SYSFS_GPIO_DIR "/gpio%d/value", gpio);
 
	fd = open(buf, O_RDONLY | O_NONBLOCK );
	if (fd < 0) {
		perror("gpio/fd_open");
	}
	return fd;
}

/****************************************************************
 * gpio_fd_close
 ****************************************************************/

int gpio_fd_close(int fd)
{
	return close(fd);
}

/****************************************************************
 * Main
 ****************************************************************/
/*int main(int argc, char **argv, char **envp)
{
	struct pollfd fdset[2];
	int nfds = 2;
	int gpio_fd, timeout, rc;
	char *buf[MAX_BUF];
	unsigned int gpio;
	int len;



	if (argc < 2) {
		printf("Usage: gpio-int <gpio-pin>\n\n");
		printf("Waits for a change in the GPIO pin voltage level or input on stdin\n");
		exit(-1);
	}

	gpio = atoi(argv[1]);

	gpio_export(gpio);
	gpio_set_dir(gpio, 0);
	gpio_set_edge(gpio, "rising");
	gpio_fd = gpio_fd_open(gpio);

	timeout = POLL_TIMEOUT;
 
	while (1) {
		memset((void*)fdset, 0, sizeof(fdset));

		fdset[0].fd = STDIN_FILENO;
		fdset[0].events = POLLIN;
      
		fdset[1].fd = gpio_fd;
		fdset[1].events = POLLPRI;

		rc = poll(fdset, nfds, timeout);      

		if (rc < 0) {
			printf("\npoll() failed!\n");
			return -1;
		}
      
		if (rc == 0) {
			printf(".");
		}
            
		if (fdset[1].revents & POLLPRI) {
			lseek(fdset[1].fd, 0, SEEK_SET);
			len = read(fdset[1].fd, buf, MAX_BUF);
			printf("\npoll() GPIO %d interrupt occurred\n", gpio);
		}

		if (fdset[0].revents & POLLIN) {
			(void)read(fdset[0].fd, buf, 1);
			printf("\npoll() stdin read 0x%2.2X\n", (unsigned int) buf[0]);
		}

		fflush(stdout);
	}

	gpio_fd_close(gpio_fd);
	return 0;
}*/