#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>

int main(void) {
	int fd = 0;
	int dst = 0;

	/* Open memdev file. */
	fd = open("/dev/memdev0", O_RDWR);
	
	/* Read data wo dst. */
	read(fd, &dst, sizeof(int));

	/* Output to console. */
	printf("dst is %d\n", dst);

	/* Close memdev file. */
	close(fd);
	
	return 0;
}


