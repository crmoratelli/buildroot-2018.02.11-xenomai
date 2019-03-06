/* Author: Carlos Roberto Moratelli*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/gpio.h>
#include <sys/mman.h>
#include <rtdm/gpiopwm.h>

#define PIN_NAME "/dev/rtdm/pinctrl-bcm2835/gpio"
#define PIN_NAME_PWM "/dev/rtdm/gpiopwm"
#define NUMPARAM 3

#define CLOCK_RES 1e-9 //Clock resolution is 1 ns by default
#define LOOP_PERIOD 1e6 //Expressed in ticks

static struct gpiopwm config;

int fd;
RT_TASK loop_task;

/* Do bitbang on gpio pins.*/
void loop_task_proc(void *arg){
	RT_TASK *curtask;
	RT_TASK_INFO curtaskinfo;
	int iret = 0;
	int value = 1;

	RTIME tstart, now;

	curtask = rt_task_self();
	rt_task_inquire(curtask, &curtaskinfo);
	int ctr = 0;

	//Print the info
	printf("Starting task %s with period of 10 ms ....\n", curtaskinfo.name);

	//Make the task periodic with a specified loop period
	rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);

	tstart = rt_timer_read();

	iret=ioctl(fd, GPIO_RTIOC_DIR_OUT, &value);

	//Start the task loop
	while(1){
		printf("Loop count: %d, Loop time: %.5f ms, value %d\n", ctr, (rt_timer_read() - tstart)/1000000.0, value);
		ctr++;
		rt_task_wait_period(NULL);
		write(fd, &value, sizeof(value));
		value = !value;
	}
}

/* Config PWM on designed pin. */
void set_pwm(char** argv){
	char n[64];
	int fd;
	int ret;

	config.gpio = atoi(argv[3]);
	config.duty_cycle =	atoi(argv[4]);
	config.range_min = atoi(argv[5]);
	config.range_max = atoi(argv[6]);
	config.period =	atoi(argv[7]);

	sprintf(n, "%s%s", PIN_NAME_PWM, argv[2]);
	printf("gpiopwm: %s, pin: %d\n", n, config.gpio);

	fd = open(n, O_RDWR);
	if(fd < 0){
		perror("open: ");
		exit(0);
	}

	ret = ioctl(fd, GPIOPWM_RTIOC_SET_CONFIG, &config);
	if (ret){
		printf("%d, %s", ret, "failed to set config");
	}

	ioctl(fd, GPIOPWM_RTIOC_START);

	ret = ioctl(fd, GPIOPWM_RTIOC_CHANGE_DUTY_CYCLE, config.duty_cycle);
	if (ret){
		printf("%d, %s", ret, "failed to set config");
	}

	printf("press key to stop..\n");
	getchar();

	ioctl(fd, GPIOPWM_RTIOC_STOP);
	close(fd);
}

void usage(char** argv){
	printf("Usage:\n");
	printf("\t%s pwm <gpiopwm [0..7]> <pin number> <duty %> <min usec> <max usec> <period nsec> \n", argv[0]);
	printf("\t\tExample: %s pwm 0 988 50 0 10000 10000000\n", argv[0]);
	printf("\t%s gpio <pin number>\n", argv[0]);
	printf("\t\tExample: %s gpio 988\n", argv[0]);
	exit(0);
}

int main(int argc, char** argv){
	char pin_name[128];
	char str[20];
	int ret;
	int pwm_value;

	if(argc < NUMPARAM){
		usage(argv);
	}

	if (!strcmp(argv[1], "pwm")){
		if (argc < 8){
			usage(argv);
		}
		set_pwm(argv);
		return 0;
	}else if (!strcmp(argv[1], "gpio")){

		sprintf(pin_name, "%s%s", PIN_NAME, argv[2]);

		printf("pin: %s\n", pin_name);

		fd = open(pin_name, O_WRONLY);
		if(fd < 0){
			perror("open: ");
			exit(0);
		}

		//Lock the memory to avoid memory swapping for this program
  		mlockall(MCL_CURRENT | MCL_FUTURE);

		printf("Starting gpio test...\n");

		//Create the real time task
		sprintf(str, "gpio_test");
		ret = rt_task_create(&loop_task, str, 1024, 50, 0);
		if(ret != 0){
			printf("rt_task_create failed %d.\n", ret);
			exit(0);
		}

		//Since task starts in suspended mode, start task
		rt_task_start(&loop_task, &loop_task_proc, 0);

		printf("press key...\n");

		//Wait for Ctrl-C
		getchar();

		printf("finished...");
	}else{
		usage(argv);	
	}

	return 0;
}

