 /**********************************************************************************
 * @file    tmpcontrol.c
 * @brief   AESD final project main program.
 *
 * @author        <Li-Huan Lu>
 * @date          <11/16/2025>
 * @reference     
 ***********************************************************************************/
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include "../lcd/lcd_ioctl.h"

#define ONE_BYTE 1
#define CONTINUOUS 0xE2 // Continuous measurement, 9-bit resolution
#define NINE_BIT   0x80
#define THREAD_NUM 3

const char btn_dev[] = "/dev/button";
const char sensor_dev[] = "/dev/ds1722";
const char lcd_dev[] = "/dev/lcd1602";

pthread_t btn_thread;
pthread_t temp_thread;
pthread_t lcd_thread;
pthread_attr_t thread_attr[THREAD_NUM];

volatile int terminate = 0;
int btn_value = 0;
int old_btn_value = 0;
int new_btn = 0;
int btn_fd, temp_fd, lcd_fd;

char sensor_buf[2];
float temp_value = 0.0;
int temp_int = 0;
float temp_decimal = 0.0;
int new_temp = 0;
float calibrate_value = 0.0;

int desired_temp = 25;
int temp_set = 1;

int new_temp2 = 0;
int new_data = 0;

int heater_on = 0;
int ac_on = 0;

char row0_str1[] = "Temp: "; // 6 characters
char row0_str2[] = ".0C";  // 3 characters
char row0_str3[] = ".5C";  // 3 characters
char row1_str1[] = "Set to "; // 7 characters
char row1_str2[] = ".0C    ";  // 7 characters
char row1_str3[] = "Heater on       ";  // 16 characters
char row1_str4[] = "AC on           ";  // 5 characters
char spaces[] = "                "; //
int row0_str1_len = 6;
int row0_str2_len = 3;
int row0_str3_len = 3;
int row1_str1_len = 7;
int row1_str2_len = 7;
int row1_str3_len = 16;
int row1_str4_len = 16;

/**********************************************************************************
 * @name       signal_handler()
 *
 * @brief      { Gracefully exits when SIGINT or SIGTERM is received. }
 *
 * @param[in]  signo
 * 
 * @return     None 
 **********************************************************************************/
void signal_handler(int signo)
{
    if (signo == SIGTERM || signo == SIGINT)
		terminate = 1;
}

/**********************************************************************************
 * @name       set_signal()       
 **********************************************************************************/
void set_signal(void)
{
	struct sigaction sa;
	
	// Set signal handler
	sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGINT, &sa, NULL);	
}

void *btn_read(void *threadp)
{
	char btn_buf;	
    ssize_t ret_byte;
	int debounce_counter = 0;
	int btn_stable = 1;
	
	btn_fd = open(btn_dev, O_RDONLY);	
	if (btn_fd == -1){
		perror("open");
		syslog(LOG_ERR, "open");
		return NULL;
	}
		
	while (!terminate){
		usleep(10000);
		if (terminate) break;
		
		ret_byte = read(btn_fd, &btn_buf, 1);
		if (ret_byte != 1){
			perror("read");
			syslog(LOG_ERR, "read");
			return NULL;
		}
		
		if (btn_stable){
			debounce_counter = 0;
			btn_value = btn_buf;
			new_btn = 1;
			btn_stable = 0;
		}
		else{
			new_btn = 0;
			
			if (btn_buf == old_btn_value){
				debounce_counter++;
			}
			if (debounce_counter >= 5){
				btn_stable = 1;
			}
			
			old_btn_value = btn_buf;
		}		
	}
	
	close(btn_fd);
	pthread_exit((void *)0);
}

void *temp_read(void *threadp)
{
	ssize_t ret_byte;
	char sensor_conf = CONTINUOUS;
	
	temp_fd = open(sensor_dev, O_RDWR);
	if (temp_fd == -1){
		perror("open");
		syslog(LOG_ERR, "open");
		return NULL;
	}
	
	// write sensor configuration
	ret_byte = write(temp_fd, &sensor_conf, 1);
	if (ret_byte != 1){
	    perror("write");
		syslog(LOG_ERR, "write");
		return NULL;
	}
	
	while (!terminate){
		sleep(3);
		if (terminate) break;
		
		// read temperature value
		ret_byte = read(temp_fd, sensor_buf, sizeof(sensor_buf));
		if (ret_byte != 2){
			perror("read");
			syslog(LOG_ERR, "read");
			return NULL;
		}
		
		// conversion
		temp_int = (int)sensor_buf[0];
		if (sensor_buf[1] == NINE_BIT)
			temp_decimal = 0.5;
		else
			temp_decimal = 0.0;
		
		temp_value = (float)temp_int + temp_decimal;
		new_temp = 1;
		//new_temp2 = 1;
	}
	
	close(temp_fd);
	pthread_exit((void *)0);
}

void *lcd_write(void *threadp)
{
	ssize_t ret_byte;
	uint8_t temp_ten_digit = 0;
	uint8_t temp_one_digit = 0;
	uint8_t desired_ten_digit = 0;
	uint8_t desired_one_digit = 0;
    char buf[2];
	
	lcd_fd = open(lcd_dev, O_WRONLY);	
	if (lcd_fd == -1){
		perror("open");
		syslog(LOG_ERR, "open");
		return NULL;
	}
	
	struct lcd_setcursor pos = {0, 0};
    ioctl(lcd_fd, LCD_IOCSETCURSOR, &pos);
	ret_byte = write(lcd_fd, row0_str1, row0_str1_len);
	if (ret_byte < 0){
		perror("write");
		syslog(LOG_ERR, "write");
		return NULL;
	}
			
	while (!terminate){
		//sleep(3);
		if (terminate) break;
		
		if (new_temp2){
			temp_ten_digit = (int)calibrate_value / 10;
			temp_one_digit = (int)calibrate_value % 10;
			
			buf[0] = temp_ten_digit + '0';
			buf[1] = temp_one_digit + '0';
			
			pos.row = 0; pos.col = 6;
	        ioctl(lcd_fd, LCD_IOCSETCURSOR, &pos);
			
			ret_byte = write(lcd_fd, buf, 2);
			
			if (temp_decimal == 0.0)
				ret_byte = write(lcd_fd, row0_str2, row0_str2_len);
			else
			    ret_byte = write(lcd_fd, row0_str3, row0_str3_len);			
			
			new_temp2 = 0;
		}	
		
		if (new_data){
			desired_ten_digit = desired_temp / 10;
			desired_one_digit = desired_temp % 10;
			
			buf[0] = desired_ten_digit + '0';
			buf[1] = desired_one_digit + '0';
			
			pos.row = 1; pos.col = 0;
	        ioctl(lcd_fd, LCD_IOCSETCURSOR, &pos);
			
			if (btn_value == 2 || btn_value == 4){ // set temp
				ret_byte = write(lcd_fd, row1_str1, row1_str1_len); // Set to
				ret_byte = write(lcd_fd, buf, 2);
				ret_byte = write(lcd_fd, row1_str2, row1_str2_len); // .0C
			}
			else{ // selected
				if (heater_on)
					ret_byte = write(lcd_fd, row1_str3, row1_str3_len);
				else if (ac_on)
					ret_byte = write(lcd_fd, row1_str4, row1_str4_len);
				else
					ret_byte = write(lcd_fd, spaces, 16);
			}
			new_data = 0;
		}
	}
	
	close(lcd_fd);
	pthread_exit((void *)0);
}	

void temp_control(void)
{
	
	switch (btn_value){
		case 1: // select
		    temp_set = 1;
			printf("Desired temperature set: %d\n", desired_temp);
		break;
		
		case 2: // up
			temp_set = 0;
			if (desired_temp < 30)
				desired_temp++;
			printf("Desired temperature: %d\n", desired_temp);
		break;
		
		case 4: // down
		    temp_set = 0;
			if (desired_temp > 16)
				desired_temp--;
			printf("Desired temperature: %d\n", desired_temp);
		break;
		
		default:
		break;
	}

	if (temp_set){
		if (desired_temp > calibrate_value){
			printf("Turn on Heater.\n");
			heater_on = 1;
			ac_on = 0;
		}	
		else if (desired_temp < calibrate_value){
			printf("Turn on AC.\n");
			heater_on = 0;
			ac_on = 1;
		}
        else{
			heater_on = 0;
			ac_on = 0;
		}
	}	
}

int main (int argc, char **argv)
{
    int ret;	
    cpu_set_t threadcpu;
	int current_state = 0;
	int prev_state = 0;
	int daemon_mode = 0;
	
	// Set up signal
	set_signal();
	
	// Check daemon mode
	if (argc == 2 && strcmp(argv[1], "-d") == 0) daemon_mode = 1;
	

	if (daemon_mode){
		pid_t pid;
	
	    pid = fork();
	
	    if (pid == -1){ // error
		    perror ("fork");
		    syslog(LOG_ERR, "Fork failed: %s", strerror(errno));
            return -1;
	    }
	    if (pid > 0){
			printf("Start in daemon\n");
			exit(0); // parent exit
		}
		
		// Child process becomes daemon
		setsid();
		chdir("/");
		// Close standard file descriptors
		close(STDIN_FILENO);
		close(STDOUT_FILENO);
		close(STDERR_FILENO);

		// Open /dev/null
		int fd = open("/dev/null", O_RDWR);
		if (fd == -1)
		{
			syslog(LOG_ERR, "Failed to open /dev/null: %s", strerror(errno));
			return -1;
		}

		// Redirect standard file descriptors to /dev/null
		dup2(fd, STDIN_FILENO);
		dup2(fd, STDOUT_FILENO);
		dup2(fd, STDERR_FILENO);
	}
	
	
	// Assign core to each thread
	for (int i = 0; i < THREAD_NUM; i++){
		pthread_attr_init(&thread_attr[i]);
		CPU_ZERO(&threadcpu);    
		CPU_SET(i, &threadcpu);	
		ret = pthread_attr_setaffinity_np(&thread_attr[i], sizeof(cpu_set_t), &threadcpu);
		if (ret < 0) perror("pthread_attr_setaffinity_np");
	}
	
	// Create threads
	ret = pthread_create(&btn_thread, &thread_attr[0], btn_read, NULL);
	if (ret){
		perror("pthread_create");
		exit(1);
	}
	else printf("pthread_create successful for btn_thread\n");
	
	ret = pthread_create(&temp_thread, &thread_attr[1], temp_read, NULL);
	if (ret){
		perror("pthread_create");
		exit(1);
	}
	else printf("pthread_create successful for temp_thread\n");
	
	ret = pthread_create(&lcd_thread, &thread_attr[2], lcd_write, NULL);
	if (ret){
		perror("pthread_create");
		exit(1);
	}
	else printf("pthread_create successful for lcd_thread\n");
	

	while (!terminate){
		if (new_btn){
			if (btn_value != 0)
				current_state = 1; // pressed
			else
				current_state = 0; // released
			
			if (current_state != prev_state){
				printf("Button value: %d\n", btn_value);
				new_data = 1;
				temp_control();
			}	
			
			prev_state = current_state;
		}	
		
		if (new_temp){
			calibrate_value = temp_value + 25.0;
			printf("Read temperature...\n");
			printf("Raw value %d %d\n", sensor_buf[0], sensor_buf[1]);
			printf("Temperature value: %.1f, Calibrated value: %0.1f\n", temp_value, calibrate_value);
			new_temp2 = 1;
			new_temp = 0;
		}
	}
	
	printf("Caught signal, exiting\n");
	syslog(LOG_DEBUG, "Caught signal, exiting\n");
	
	pthread_join(btn_thread, NULL);
	pthread_join(temp_thread, NULL);
	pthread_join(lcd_thread, NULL);
	
	if (btn_fd != -1) close(btn_fd);
	if (temp_fd != -1) close(temp_fd);
	if (lcd_fd != -1) close(lcd_fd);
	
	return 0;
}