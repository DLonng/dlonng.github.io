#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

int main(void) {
	int fd = 0;
	int src = 2017;
	
	/* Open memdev file. */
	fd = open("/dev/memdev0", O_RDWR);

	/* Write data to memedev. */
	write(fd, &src, sizeof(int));

	/* Close memdev file. */
	close(fd);
	return 0;
}


