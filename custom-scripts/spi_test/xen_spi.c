#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <rtdm/gpio.h>
#include <sys/mman.h>
#include <rtdm/gpiopwm.h>
#include <rtdm/uapi/spi.h>
#include <math.h>
#include <semaphore.h>

#define RUN_STATISTICS

#define DEVICE_SPI "/dev/rtdm/spi32766/slave32766.0"
#define PIN_NAME_PWM "/dev/rtdm/gpiopwm0"

#define TRANSFER_SIZE 6
#define CLOCK_RES 1e-9 /* Clock resolution is 1ns */
#define DIV_TO_MS  1000000.0
#define DIV_TO_US  1000.0
#define LOOP_PERIOD 100000 /* Expressed in ticks - 100 us */

struct rtdm_spi_iobufs iobufs;

/* Power model data */
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

#ifdef RUN_STATISTICS

/* Statistics */
struct time_data{
	unsigned int  ctr;
	double passed_time;
	double loop_time;
	double jitter;
	int value;
	int pwm;
};

#define INFO_SIZE 1000000
struct time_data loop_info[INFO_SIZE];
double time_cases[4] = {9999999, -1, 9999999, -1};
#define FASTEST_LOOP 0
#define SLOWER_LOOP 1
#define BETTER_JITTER 2
#define WORST_JITTER 3
sem_t sem;
#else
volatile int finish = 1;

#endif /* RUN_STATISTICS */

double map(double x, double in_min, double in_max, double out_min, double out_max){
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Perform power model equation. */
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

/* Configure the PWM controller. */
int configure_pwm_pin(){
	struct gpiopwm config_pwm;
	int fd;
	void *p;
	int ret;
	

	config_pwm.gpio = 988;
	config_pwm.duty_cycle =	0;
	config_pwm.range_min = 0;
	config_pwm.range_max = 1000;
	config_pwm.period =	1000000;

	fd = open(PIN_NAME_PWM, O_RDWR);
	if(fd < 0){
		perror("open: ");
		exit(0);
	}

	ret = ioctl(fd, GPIOPWM_RTIOC_SET_CONFIG, &config_pwm);
	if (ret){
		printf("%d, %s", ret, "failed to set pwm config.");
		exit(0);
	}

	ret = ioctl(fd, GPIOPWM_RTIOC_START);
	if (ret){
		printf("%d, %s", ret, "failed to start pwm.");
		exit(0);
	}

	return fd;
}

/* Configure the SPI controller. */
int configure_spi_device(unsigned char **i_area, unsigned char **o_area){
	struct rtdm_spi_config config = {3600000, 0, 8};
	int fd;
	void *p;
	int ret;

	fd = open(DEVICE_SPI, O_RDWR);
	if (fd < 0){
		perror("open(spi):");
	}

	 /* spi mode  */
	ret = ioctl(fd, SPI_RTIOC_SET_CONFIG, &config);
	if (ret == -1){
		printf("ioctl(spi) failed %d\n", ret);
	}

	printf("spi mode: %d\n", config.mode);
	printf("bits per word: %d\n", config.bits_per_word);
	printf("max speed: %d Hz (%d KHz)\n", config.speed_hz, config.speed_hz/1000);

	iobufs.io_len = TRANSFER_SIZE;

	ret = ioctl(fd, SPI_RTIOC_SET_IOBUFS, &iobufs);
	if(ret < 0){
		perror("ioctl()");
	}

	p = mmap(NULL, iobufs.io_len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED){
		printf("mmap() failed. \n");
		exit(0);
	}

	*i_area = p + iobufs.i_offset;
	*o_area = p + iobufs.o_offset;
	
	printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u\n",
		     iobufs.i_offset, iobufs.i_offset + TRANSFER_SIZE - 1,
		     iobufs.o_offset, iobufs.o_offset + TRANSFER_SIZE - 1,
		     iobufs.io_len);

	return fd;
}

#ifdef RUN_STATISTICS
/* Update statistic tables */
void update_statistics(int ctr, double passed_time, double loop_time, double jitter, int value, int pwm){
	loop_info[ctr%INFO_SIZE].ctr = ctr;
	loop_info[ctr%INFO_SIZE].passed_time = passed_time;
	loop_info[ctr%INFO_SIZE].loop_time = loop_time;
	loop_info[ctr%INFO_SIZE].jitter = jitter;
	loop_info[ctr%INFO_SIZE].value = value;
	loop_info[ctr%INFO_SIZE].pwm = pwm;

	if (ctr > 50){
		if(time_cases[FASTEST_LOOP] > loop_info[ctr%INFO_SIZE].loop_time){
			time_cases[FASTEST_LOOP] = loop_info[ctr%INFO_SIZE].loop_time;
		}

		if(time_cases[SLOWER_LOOP] < loop_info[ctr%INFO_SIZE].loop_time){
			time_cases[SLOWER_LOOP] = loop_info[ctr%INFO_SIZE].loop_time;
		}

		if(time_cases[BETTER_JITTER] > loop_info[ctr%INFO_SIZE].jitter){
			time_cases[BETTER_JITTER] = loop_info[ctr%INFO_SIZE].jitter;
		}

		if(time_cases[WORST_JITTER] < loop_info[ctr%INFO_SIZE].jitter){
			time_cases[WORST_JITTER] = loop_info[ctr%INFO_SIZE].jitter;
		}
	}
}
#endif


/* Real-time Task */
void loop_task_proc(void *arg){
	RT_TASK *curtask;
	RT_TASK_INFO curtaskinfo;
	RTIME tstart, tlast=0, now;
	int ret = 0;
	int value = 1;
	int pwm, old_pwm;
	int fdpwm, fdspi;
	int ctr = 0;
	unsigned char *i_area, *o_area;

	curtask = rt_task_self();
	rt_task_inquire(curtask, &curtaskinfo);

	fdpwm = configure_pwm_pin();

	fdspi = configure_spi_device(&i_area, &o_area);

	printf("Starting task %s with period of %d us ....\n", curtaskinfo.name, LOOP_PERIOD/1000);

	/* Make the task periodic with a specified loop period */
	rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);

	/* Data to send to the MCP3008 */
	o_area[0] = 0x1;
	o_area[1] = 0x80;
	o_area[2] = 0;

	/* Initial time. */
	tstart = rt_timer_read();

#ifdef RUN_STATISTICS
	while(ctr<INFO_SIZE){
#else
	while(finish){
#endif		
		now = rt_timer_read();
		ctr++;

		/* Data tranfer to the MPC3008 */
		ret =ioctl(fdspi, SPI_RTIOC_TRANSFER);
		if(ret < 0){
			perror("ioctl");
		}

		/* Read the returned data. */
		value = (i_area[1] & 0x7) << 8 | i_area[2];

		/* Performe the control model equations. */
		pwm = do_control(value, (rt_timer_read() - tstart));

		/* Adjust PWM if nedded */
		if(pwm != old_pwm){
			ioctl(fdpwm, GPIOPWM_RTIOC_CHANGE_DUTY_CYCLE, pwm);
			old_pwm = pwm;
		}

#ifdef	RUN_STATISTICS
		/* keep statistics */
		update_statistics(ctr, (now - tstart)/DIV_TO_MS, (rt_timer_read()-now)/DIV_TO_US, ((long int)now - (long int)tlast - LOOP_PERIOD), value, pwm);
#endif

		tlast = now;		

		rt_task_wait_period(NULL);
	}

	close(fdspi);
	close(fdpwm);

#ifdef RUN_STATISTICS
	sem_post(&sem);
#endif
}


#ifdef RUN_STATISTICS
/*Print statistics */
void print_results(){
	int i;

	printf("Fastest Loop(us)\tSlower Loop(us)\t\tBetter Jitter(us)\tWorst Jitter(us)\n");
	printf("%.2f\t\t\t%.2f\t\t\t%.2f\t\t\t%.2f\n\n", time_cases[FASTEST_LOOP], time_cases[SLOWER_LOOP], time_cases[BETTER_JITTER], time_cases[WORST_JITTER]);

	printf("Loop count\trun time(ms)\tloop time(us)\tjitter(us)\tanalog\t\tpwm\n");

	for(i=0; i<INFO_SIZE; i++){
		printf("%d\t\t%.2f\t%.2f\t\t%.5f\t\t%d\t\t%d\n", loop_info[i].ctr, loop_info[i].passed_time, loop_info[i].loop_time, loop_info[i].jitter, loop_info[i].value, loop_info[i].pwm);
	}
}
#endif


int main(int argc, char *argv[]){
	RT_TASK loop_task;
	char str[20];
	int ret;

#ifdef RUN_STATISTICS
	memset(loop_info, 0, sizeof(loop_info));

	printf("loop_info size: %dKB\n", sizeof(loop_info)/1024);

	/* Mutex for wait finish */
	sem_init(&sem, 0 , 0);
#endif 	

	/* Create the real time task */
	sprintf(str, "spi_loop");
	ret = rt_task_create(&loop_task, str, 1024, 99, 0);
	if(ret != 0){
		printf("rt_task_create failed %d.\n", ret);
		exit(0);
	}

	/* Since task starts in suspended mode, start task */
	rt_task_start(&loop_task, &loop_task_proc, 0);

#ifdef RUN_STATISTICS
	printf("Wait... ");

	/* Wait main RT thread to finish*/
	sem_wait(&sem);
#else
	printf("Press ENTER to finish.");
	getchar();
#endif	

	printf("finished...\n");

#ifdef RUN_STATISTICS
	print_results();
#endif 

	return ret;
}

