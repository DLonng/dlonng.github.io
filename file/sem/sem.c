#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>

// 使用信号量必须定义这个结构
union semun {  
	int val;  
	struct semid_ds *buf;  
	unsigned short *arry;  
};


// 信号量的 p 操作，减 1 加锁
struct sembuf sem_lock   = { 0, -1, 0 };

// 信号量的 v 操作，加 1 解锁
struct sembuf sem_unlock = { 0,  1, 0 };

// 创建或获取一个信号量
int sem_get(int sem_key) {
	int sem_id = semget(sem_key, 1, IPC_CREAT | 0666);
	
	if (sem_id == -1) {
		printf("sem get failed.\n");
		exit(-1);
	} else {
		printf("sem_id = %d\n", sem_id);
		return sem_id;
	}	
}

// 初始化信号量
int set_sem(int sem_id) {
	union semun sem_union;  
	sem_union.val = 1;  
	if(semctl(sem_id, 0, SETVAL, sem_union) == -1) { 
		fprintf(stderr, "Failed to set sem\n");  
		return 0;  
	}

	return 1;  
}

// 删除信号量  
void del_sem(int sem_id) {  
	union semun sem_union;  
	if(semctl(sem_id, 0, IPC_RMID, sem_union) == -1)  
		fprintf(stderr, "Failed to delete sem, sem has been del.\n");  
} 

// 加锁
void sem_down(int sem_id) {
	if (-1 == semop(sem_id, &sem_lock, 1)) 
		fprintf(stderr, "semaphore lock failed.\n");
}

// 解锁
void sem_up(int sem_id) {
	if (-1 == semop(sem_id, &sem_unlock, 1))
		fprintf(stderr, "semaphore unlock failed.\n");
}



int main(int argc, char **argv) {
	int sem_id = sem_get(12);

	// 第一次调用多加一个参数，第二次调用不加参数，仅在第一次调用时创建信号量
	if (argc > 1 && (!set_sem(sem_id))) {
		printf("set sem failed.\n");
		return -1;
	}

	sem_down(sem_id);
	printf("sem lock...\n");
	
	printf("do something...\n");
	sleep(10);

	sem_up(sem_id);
	printf("sem unlock...\n");

	// 第二次调用后删除信号量
	if (argc == 1)
		del_sem(sem_id);	

	return 0;
}
