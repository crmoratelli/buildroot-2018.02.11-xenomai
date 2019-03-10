#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/gpio.h>
#include <sys/mman.h>
#include <rtdm/gpiopwm.h>
#include <rtdm/uapi/spi.h>

#define CHANNEL(x) (unsigned char)(x << 4)

static const char *device = "/dev/rtdm/spi32766/slave32766.0";

#define TRANSFER_SIZE 3
#define CLOCK_RES 1e-9 //Clock resolution is 1 ns by default
#define LOOP_PERIOD 33333333 //Expressed in ticks

struct rtdm_spi_config config = {1000000, 0, 8};
struct rtdm_spi_iobufs iobufs;
void *p;
static unsigned char *i_area, *o_area;
int fd;

RT_TASK loop_task;


/* Do bitbang on gpio pins.*/
void loop_task_proc(void *arg){
	RT_TASK *curtask;
	RT_TASK_INFO curtaskinfo;
	int iret = 0;
	int value = 1;

	RTIME tstart, tlast=0, now;

	curtask = rt_task_self();
	rt_task_inquire(curtask, &curtaskinfo);
	int ctr = 0;

	//Print the info
	printf("Starting task %s with period of 10 ms ....\n", curtaskinfo.name);

	//Make the task periodic with a specified loop period
	rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);

	tstart = rt_timer_read();

	//Start the task loop
	while(1){
		now = rt_timer_read();
		printf("Loop count: %d, Loop time: %.5f ms, jitter %.5f us\n", ctr, (now - tstart)/1000000.0, ((long int)now - (long int)tlast - LOOP_PERIOD)/1000.0);
		tlast = now;		
		ctr++;


		o_area[0] = 0x1;
		o_area[1] = CHANNEL(ctr%8);
		o_area[2] = 0;

		iret =ioctl(fd, SPI_RTIOC_TRANSFER);
		if(iret < 0){
			perror("ioctl");
		}

		/* */

		rt_task_wait_period(NULL);
	}
}


int main(int argc, char *argv[])
{
	int ret = 0;
	char str[20];

	fd = open(device, O_RDWR);
	if (fd < 0){
		perror("open():");
	}

	/*
	 * spi mode
	 */
	ret = ioctl(fd, SPI_RTIOC_SET_CONFIG, &config);
	if (ret == -1){
		perror("ioctl():");
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
		return -EINVAL;
	}

	i_area = p + iobufs.i_offset;
	o_area = p + iobufs.o_offset;
	
	printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u\n",
		     iobufs.i_offset, iobufs.i_offset + TRANSFER_SIZE - 1,
		     iobufs.o_offset, iobufs.o_offset + TRANSFER_SIZE - 1,
		     iobufs.io_len);
	
	printf("Starting loop...\n");

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

