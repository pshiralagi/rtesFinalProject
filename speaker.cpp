#include <pigpio.h>

#include <stdio.h>
#include <stdlib.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>

#define LOW 100000
#define MEDIUM 250000
#define HIGH 50000
#define OFF 0

void ipc();

typedef struct {
	int unique_id;
	int tmp;
} data;

const int PWM_pin = 17;

void pwm_pulse(int frequency){
    int i=0;
    if(frequency == 0){
      gpioWrite(PWM_pin, PI_OFF);
    }else if(frequency == HIGH){
      while(i < 100){
	gpioWrite(PWM_pin, PI_ON);
	gpioDelay(HIGH);
	gpioWrite(PWM_pin, PI_OFF);
	gpioDelay(HIGH);
	printf("%d\n", i);
	i++;
      }
    }else{
	gpioWrite(PWM_pin, PI_ON);
	gpioDelay(frequency);
	gpioWrite(PWM_pin, PI_OFF);
	gpioDelay(frequency);
    }
    i = 0;
    gpioWrite(PWM_pin, PI_OFF);
}


int main(){
	gpioInitialise();
	gpioSetMode (PWM_pin, PI_OUTPUT);
	
	while (1)
	{
		ipc();
	}
	 
}

/*	Function to access shared memory	*/
void ipc(void)
{
	static int pulse;
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
	printf("Temperature is %d\n\r", temp_ptr->tmp);
	pulse = temp_ptr->tmp;
	munmap(temp_ptr,sizeof(data));
	printf("Pulse -- %d", pulse);
	switch (pulse)
	{
	case 0:
		pwm_pulse(OFF);
		break;
	case 1:
		pwm_pulse(LOW);
		break;
	case 2:
		pwm_pulse(MEDIUM);
		break;
	case 3:
		pwm_pulse(HIGH);
		break;
	default:
		break;
	}
	
}
