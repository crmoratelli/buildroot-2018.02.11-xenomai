Buildroot with Linux Kernel 4.9.51 patched with I-Pipe and Xenomai v3.1-devel libraries.


To build:

mkdir dl/
cp custom-scripts/packages/linux-xenomai.tar.gz dl/
make raspberrypi3_xenomai_defconfig
make

To test GPIO and PWM using RTDM drivers:

/usr/xenomai/bin/xen-gpio pwm <gpiopwm [0..7]> <pin number> <duty %> <min usec> <max usec> <period nsec> 
    Example: /usr/xenomai/bin/xen-gpio pwm 0 988 50 0 10000 10000000
/usr/xenomai/bin/xen-gpio gpio <pin number>
    Example: /usr/xenomai/bin/xen-gpio gpio 988
