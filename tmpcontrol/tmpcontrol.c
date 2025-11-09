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
const char btn_dev[] = "/dev/button";
int main ()
{
    int fd;
	int btn_value = 0;
    ssize_t ret_byte;
	char btn_buf[ONE_BYTE];
	
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
	
	return 0;
}