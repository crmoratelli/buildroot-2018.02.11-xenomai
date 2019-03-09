#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <rtdm/gpio.h>
#include <sys/mman.h>
#include <rtdm/gpiopwm.h>
#include <rtdm/uapi/spi.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

static const char *device = "/dev/rtdm/spi32766/slave32766.0";

#define TRANSFER_SIZE 8
struct rtdm_spi_config config = {1000000, 0, 8};
struct rtdm_spi_iobufs iobufs;
void *p;
static unsigned char *i_area, *o_area;


int main(int argc, char *argv[])
{
	int ret = 0;
	int fd;

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


	iobufs.io_len = TRANSFER_SIZE;

	ret = ioctl(fd, SPI_RTIOC_SET_IOBUFS, &iobufs);
	if(ret < 0){
		perror("ioctl()");
	}


	p = mmap(NULL, iobufs.io_len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED){
		return -EINVAL;
	}
	
	printf("input_area[%u..%u], output_area[%u..%u], mapping length=%u\n",
		     iobufs.i_offset, iobufs.i_offset + TRANSFER_SIZE - 1,
		     iobufs.o_offset, iobufs.o_offset + TRANSFER_SIZE - 1,
		     iobufs.io_len);
	
	i_area = p + iobufs.i_offset;
	o_area = p + iobufs.o_offset;

	o_area[0] = 0xff;
	o_area[1] = 0xfe;
	o_area[2] = 0xff;
	o_area[3] = 0xf5;

	ret =ioctl(fd, SPI_RTIOC_TRANSFER);
	if(ret < 0){
		perror("ioctl");
	}

	printf("spi mode: %d\n", config.mode);
	printf("bits per word: %d\n", config.bits_per_word);
	printf("max speed: %d Hz (%d KHz)\n", config.speed_hz, config.speed_hz/1000);

	//transfer(fd);

	close(fd);

	return ret;
}

