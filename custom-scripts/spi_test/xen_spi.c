#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/gpio.h>
#include <sys/mman.h>
#include <rtdm/gpiopwm.h>
#include <rtdm/uapi/spi.h>
#include <math.h>

#define CHANNEL(x) (unsigned char)(x << 4)

static const char *device = "/dev/rtdm/spi32766/slave32766.0";
#define PIN_NAME_PWM "/dev/rtdm/gpiopwm0"

#define TRANSFER_SIZE 6
#define CLOCK_RES 1e-9 /* Clock resolution is 1ns */
#define DIV_TO_MS  1000000.0
#define DIV_TO_US  1000.0
#define LOOP_PERIOD 100000000 /* Expressed in ticks - 100 ms */

struct rtdm_spi_config config = {3000000, 0, 8};
struct rtdm_spi_iobufs iobufs;
void *p;
static unsigned char *i_area, *o_area;
int fd;

RT_TASK loop_task;

/* */
const double coef_nom[4] = {0.2039,   -0.4210,    0.2811,   -0.0608};

const double coef_den[4] = {1.0000,   -1.6919,    0.3841,    0.3080};

double y = 0; //Valor mensurado pelo sensor.
double e = 0; //Valor da diferenca entre a referencia e o valor de y
double u = 0; //sinal de controle.

double u_n1 = 0; //Sinal de controle de uma amostra atrasada.
double u_n2 = 0; //Sinal de controle de duas amostras atrasada.
double u_n3 = 0; //Sinal de controle de tres amostras atrasada.

double e_n1 = 0; //Sinal de erro de uma amostra atrasada.
double e_n2 = 0; //Sinal de erro de duas amostras atrasada.
double e_n3 = 0; //Sinal de erro de tres amostras atrasada.

static struct gpiopwm config_pwm;// = {988, 50, 0, 1000000, 1000000};


double map(double x, double in_min, double in_max, double out_min, double out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int do_control(int v, unsigned int t){
	double u, ref;

	ref = 2*90/216*sin(377*t); //Referencia

	u_n1 = u;
	u_n2 = u_n1;	
	u_n3 = u_n2;

	e_n1 = e;
	e_n2 = e_n1;
	e_n3 = e_n2;

	e = ref - v;

	u = coef_den[0]*e + coef_den[1]*e_n1 + coef_den[2]*e_n2 + coef_den[3] - coef_nom[0]*u_n1 - coef_nom[1]*u_n2 - coef_nom[2]*u_n2 - coef_nom[3];  

	u = map(u, 0, 315, 0, 100);

	return (int)u;
}


/* Do bitbang on gpio pins.*/
void loop_task_proc(void *arg){
	RT_TASK *curtask;
	RT_TASK_INFO curtaskinfo;
	int iret = 0;
	int value = 1;
	int pwm;
	int fdpwm;

	RTIME tstart, tlast=0, now;

	curtask = rt_task_self();
	rt_task_inquire(curtask, &curtaskinfo);
	int ctr = 0;

	config_pwm.gpio = 988
	config_pwm.duty_cycle =	atoi(argv[4]);
	config_pwm.range_min = atoi(argv[5]);
	config_pwm.range_max = atoi(argv[6]);
	config_pwm.period =	atoi(argv[7]);


	fdpwm = open(PIN_NAME_PWM, O_RDWR);
	if(fdpwm < 0){
		perror("open: ");
		exit(0);
	}

	iret = ioctl(fdpwm, GPIOPWM_RTIOC_SET_CONFIG, &config_pwm);
	if (iret){
		printf("%d, %s", iret, "failed to set config");
	}

	ioctl(fdpwm, GPIOPWM_RTIOC_START);

	fd = open(device, O_RDWR);
	if (fd < 0){
		perror("open():");
	}

	/*
	 * spi mode
	 */
	iret = ioctl(fd, SPI_RTIOC_SET_CONFIG, &config);
	if (iret == -1){
		perror("ioctl():");
	}

	printf("spi mode: %d\n", config.mode);
	printf("bits per word: %d\n", config.bits_per_word);
	printf("max speed: %d Hz (%d KHz)\n", config.speed_hz, config.speed_hz/1000);

	iobufs.io_len = TRANSFER_SIZE;

	iret = ioctl(fd, SPI_RTIOC_SET_IOBUFS, &iobufs);
	if(iret < 0){
		perror("ioctl()");
	}

	p = mmap(NULL, iobufs.io_len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED){
		return;
	}

	i_area = p + iobufs.i_offset;
	o_area = p + iobufs.o_offset;
	
	printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u\n",
		     iobufs.i_offset, iobufs.i_offset + TRANSFER_SIZE - 1,
		     iobufs.o_offset, iobufs.o_offset + TRANSFER_SIZE - 1,
		     iobufs.io_len);

	//Print the info
	printf("Starting task %s with period of %.2f ms ....\n", curtaskinfo.name, LOOP_PERIOD/1000000);

	//Make the task periodic with a specified loop period
	rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);

	o_area[0] = 0x1;
	o_area[1] = 0x80;
	o_area[2] = 0;

	tstart = rt_timer_read();

	//Start the task loop
	while(1){
		now = rt_timer_read();
		ctr++;

		iret =ioctl(fd, SPI_RTIOC_TRANSFER);
		if(iret < 0){
			perror("ioctl");
		}

		value = (i_area[1] & 0x7) << 8 | i_area[2];
		printf("value: %d\n", value);

		pwm = do_control(value, (rt_timer_read() - tstart));
		printf("pwm: %d\n", pwm);

		ioctl(fd, GPIOPWM_RTIOC_CHANGE_DUTY_CYCLE, pwm);

		printf("Loop count: %d, run time: %.2f ms, loop time: %.2f us jitter %.5f us\n", ctr, (now - tstart)/DIV_TO_MS, (rt_timer_read()-now)/DIV_TO_US, ((long int)now - (long int)tlast - LOOP_PERIOD)/DIV_TO_US);
		tlast = now;		

		rt_task_wait_period(NULL);
	}
}


int main(int argc, char *argv[]){
	char str[20];
	int ret;

	//Create the real time task
	sprintf(str, "spi_loop");
	ret = rt_task_create(&loop_task, str, 1024, 99, 0);
	if(ret != 0){
		printf("rt_task_create failed %d.\n", ret);
		exit(0);
	}

	//Since task starts in suspended mode, start task
	rt_task_start(&loop_task, &loop_task_proc, 0);

	printf("press key to finish...\n");

	//Wait for Ctrl-C
	getchar();

	printf("finished...");

	close(fd);

	return ret;
}

