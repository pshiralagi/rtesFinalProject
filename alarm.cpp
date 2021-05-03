/*
//	@filename: alarm.cpp
//	@description: This process receives alarm information from the finalProject.cpp file and prints relevant statements
*/

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>


void ipc();

typedef struct {
	int unique_id;
	int tmp;
} data;


int main(){
	while (1)
	{
		ipc();
	}
	 
}

/*	Function to access shared memory	*/
void ipc(void)
{
	static int pulse = 0;
	data temperature;
	data *temperature_ptr = &temperature;
	data *temp_ptr = NULL;
	int fd_shared;
	fd_shared = shm_open("shareTmp", O_RDONLY, 0666);
	if (fd_shared < 0)
	{
		perror("open");
	}
	temp_ptr = (data *)mmap(NULL, sizeof(data), PROT_READ, MAP_SHARED, fd_shared, 0);
	if (close(fd_shared) < 0)
	{
		perror("close");
	}
	memcpy((void*)temperature_ptr,(void*)(&temp_ptr[0]),sizeof(data));
	pulse = temp_ptr->tmp;
	munmap(temp_ptr,sizeof(data));

	switch (pulse)
	{
	case 0:
		printf("Situation Safe\n\r");
		break;
	case 1:
		printf("Initial detection of person at doorway, start real time processes\n\r");
		break;
	case 2:
		printf("Person is within the camera viewing angle, should detect the circle any moment now!\n\r");
		break;
	case 3:
		printf("Circle not detected!\n\r");
		break;
	default:
		break;
	}
	
}
