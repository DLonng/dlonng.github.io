#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>


/*
 * child write, parent read
 */
int main() {
	int pfd[2];
	int pid;
	int status = 0;

	char w_cont[] = "Hello child, I'm child!";
	char r_cont[255] = { 0 }; 
	
	if(pipe(pfd) < 0) {
		perror("create pipe failed"); 
		exit(1); 
	} else {
		if((pid = fork()) < 0) {
			perror("create process failed");
		} else if(pid > 0) {
			close(pfd[1]); 
			read(pfd[0], r_cont, strlen(w_cont)); 
			close(pfd[0]);
			wait(&status);
			printf("parent process read: %s\n", r_cont); 
		} else {
			close(pfd[0]); 
			write(pfd[1], w_cont, strlen(w_cont)); 
			close(pfd[1]);
		}
	}

	return 0 ; 
}
