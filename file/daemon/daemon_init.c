#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/resource.h>

void daemon_init(void) {
 	// 1. 重新设置 umask
	umask(0); 
	
	// 2. 调用 fork 并脱离父进程
	pid_t pid = fork()  ; 
	
	if(pid < 0)
		exit(1); 
	else if(pid > 0)
		exit(0);

	// 3. 重启 session 会话
	setsid();

	// 4. 改变工作目录
	chdir("/"); 

	// 5. 得到并关闭文件描述符
	struct rlimit rl;
	getrlimit(RLIMIT_NOFILE, &rl);
	if (rl.rlim_max == RLIM_INFINITY)
		rl.rlim_max = 1024;
	for(int i = 0; i < rl.rlim_max; i++)	
		close(i); 
	
	// 6. 不接受标准输入，输入，错误
	int fd0 = open("/dev/null", O_RDWR);
	int fd1 = dup(0);
	int fd2 = dup(0);	 
}

