STAGE=../../output/target/
XENO_CONFIG := ../../output/target/usr/xenomai/bin/xeno-config
CFLAGS := $(shell DESTDIR=$(STAGE) $(XENO_CONFIG) --rtdm --skin=alchemy --cflags)
LDFLAGS := $(shell DESTDIR=$(STAGE) $(XENO_CONFIG) --rtdm --skin=alchemy --ldflags)
CC := $(shell DESTDIR=$(STAGE) $(XENO_CONFIG) --cc)

EXECUTABLE := xen-gpio


all: $(EXECUTABLE)

%: %.c
	$(CC) -o $@ $< $(CFLAGS) $(LDFLAGS)


