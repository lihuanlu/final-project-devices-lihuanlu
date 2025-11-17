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
//#include <string.h>
#include <syslog.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <sched.h>

#define ONE_BYTE 1
#define CONTINUOUS 0xE2 // Continuous measurement, 9-bit resolution
#define NINE_BIT   0x80

const char btn_dev[] = "/dev/button";
const char sensor_dev[] = "/dev/ds1722";

pthread_t btn_thread;
pthread_t temp_thread;
pthread_attr_t thread_attr;

volatile int terminate = 0;
int btn_value = 0;
int old_btn_value = 0;
int new_btn = 0;
int btn_fd, temp_fd;

char sensor_buf[2];
float temp_value = 0.0;
int temp_int = 0;
float temp_decimal = 0.0;
int new_temp = 0;
float calibrate_value = 0.0;

int desired_temp = 25;
int temp_set = 1;
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

    int last_raw = 0;
    int stable_raw = 0;
    int debounce_counter = 0;

    btn_fd = open(btn_dev, O_RDONLY);
    if (btn_fd == -1) {
        perror("open");
        return NULL;
    }

    while (!terminate) {

        usleep(50000);  // 50ms

        ret_byte = read(btn_fd, &btn_buf, 1);
        if (ret_byte != 1)
            continue;

        int raw = btn_buf & 0x07;

        // --- debounce: check if consecutive samples are identical ---
        if (raw == last_raw) {
            debounce_counter++;
        } else {
            debounce_counter = 0;
        }

        // stable if same reading for 3 consecutive samples
        if (debounce_counter == 3) {

            // --- detect a stable rising edge (0 -> nonzero) ---
            if (stable_raw == 0 && raw != 0) {
                btn_value = raw;
                new_btn = 1;        // trigger event
            }

            stable_raw = raw; // update stable state
        }

        last_raw = raw;
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
	}
	
	close(temp_fd);
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
		if (desired_temp > calibrate_value)
			printf("Turn on Heater.\n");
		if (desired_temp < calibrate_value)
			printf("Turn on AC.\n");
	}	
}

int main ()
{
    int ret;	
    cpu_set_t threadcpu;
	
	// Set up signal
	set_signal();
	
	pthread_attr_init(&thread_attr);
	CPU_ZERO(&threadcpu);    
    CPU_SET(0, &threadcpu);	
	ret = pthread_attr_setaffinity_np(&thread_attr, sizeof(cpu_set_t), &threadcpu);
	if (ret < 0) perror("pthread_attr_setaffinity_np");
	
	ret = pthread_create(&btn_thread, &thread_attr, btn_read, NULL);
	if (ret){
		perror("pthread_create");
		exit(1);
	}
	else printf("pthread_create successful for btn_thread\n");
	
	ret = pthread_create(&temp_thread, NULL, temp_read, NULL);
	if (ret){
		perror("pthread_create");
		exit(1);
	}
	else printf("pthread_create successful for temp_thread\n");
	
	while (!terminate){
		if (btn_value != 0 && new_btn){
			printf("Button value: %d\n", btn_value);
			temp_control();
		}	
		
		if (new_temp){
			calibrate_value = temp_value + 25.0;
			printf("Read temperature...\n");
			printf("Raw value %d %d\n", sensor_buf[0], sensor_buf[1]);
			printf("Temperature value: %.1f, Calibrated value: %0.1f\n", temp_value, calibrate_value);
			new_temp = 0;
		}
	}
	
	printf("Caught signal, exiting\n");
	syslog(LOG_DEBUG, "Caught signal, exiting\n");
	
	pthread_join(btn_thread, NULL);
	pthread_join(temp_thread, NULL);
	
	if (btn_fd != -1) close(btn_fd);
	if (temp_fd != -1) close(temp_fd);
	
	return 0;
}