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

#define TRANSFER_SIZE 6
#define CLOCK_RES 1e-9 /* Clock resolution is 1ns */
#define DIV_TO_MS  1000000.0
#define DIV_TO_US  1000.0
#define LOOP_PERIOD 100000000 /* Expressed in ticks - 100 ms */

struct rtdm_spi_config config = {1500000, 0, 8};
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

