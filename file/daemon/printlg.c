#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

void daemon_init(void);

int main(void) {
	daemon_init(); 

	char *msg = "I'm daemon process...\n" ;
	
	int fd = open("/tmp/test_daemon.log", O_RDWR | O_CREAT | O_APPEND, 0666); 
	if(fd < 0) { 
		printf("open /tmp/test_daemon.log fail.\n");
		exit(1);
	}

	while(1) {
		write(fd, msg, strlen(msg))  ; 
		sleep(3); 
	}

	close(fd);
	return 0;
}

