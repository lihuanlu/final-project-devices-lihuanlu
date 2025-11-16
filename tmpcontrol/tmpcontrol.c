 /**********************************************************************************
 * @file    tmpcontrol.c
 * @brief   AESD final project main program.
 *
 * @author        <Li-Huan Lu>
 * @date          <11/09/2025>
 * @reference     
 ***********************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
//#include <string.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
//#include <signal.h>
//#include <time.h>
#include <errno.h>

#define ONE_BYTE 1
#define CONTINUOUS 0xE2 // Continuous measurement, 9-bit resolution
#define NINE_BIT   0x80

const char btn_dev[] = "/dev/button";
const char sensor_dev[] = "/dev/ds1722";

int main ()
{
    int fd;
	int btn_value = 0;
    ssize_t ret_byte;
	char btn_buf[ONE_BYTE];
	char sensor_buf[2];
	char sensor_conf[ONE_BYTE] = CONTINUOUS;
	float temp_value = 0.0;
	int temp_int = 0;
	float temp_decimal = 0.0;
	
    fd = open(btn_dev, O_RDONLY);	
	if (fd == -1){
		perror("open");
		syslog(LOG_ERR, "open");
		return -1;
	}
		
	ret_byte = read(fd, btn_buf, sizeof(btn_buf));
	if (ret_byte != 1){
		if (ret_byte == 0)
			printf("EOF\n");
		
		perror("read");
		syslog(LOG_ERR, "read");
		return -1;
	}
	
	btn_value = (int)btn_buf[0];
	printf("Button value: %d\n", btn_value);
	
	close(fd);
	
	fd = open(sensor_dev, O_RDWR);
	if (fd == -1){
		perror("open");
		syslog(LOG_ERR, "open");
		return -1;
	}
	
	// write sensor configuration
	ret_byte = write(fd, sensor_conf, sizeof(sensor_conf));
	if (ret_byte != 1){
	    perror("write");
		syslog(LOG_ERR, "write");
		return -1;
	}
	
	// read temperature value
	ret_byte = read(fd, sensor_buf, sizeof(sensor_buf));
	if (ret_byte != 2){
		perror("read");
		syslog(LOG_ERR, "read");
		return -1;
	}
	
	temp_int = (int)sensor_buf[0];
	if (sensor_buf[0] == NINE_BIT)
		temp_decimal = 0.5;
	else
		temp_decimal = 0.0;
	
	temp_value = (float)temp_int + temp_decimal;
	printf("Temperature value: %f\n", temp_value);
	
	return 0;
}