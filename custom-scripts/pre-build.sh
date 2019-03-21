#!/bin/bash
  
cp $BASE_DIR/../custom-scripts/S41network-config $BASE_DIR/target/etc/init.d
chmod +x $BASE_DIR/target/etc/init.d/S41network-config

cp $BASE_DIR/../custom-scripts/S50load-drivers $BASE_DIR/target/etc/init.d
chmod +x $BASE_DIR/target/etc/init.d/S50load-drivers

cp $BASE_DIR/../custom-scripts/S52autotune $BASE_DIR/target/etc/init.d
chmod +x $BASE_DIR/target/etc/init.d/S52autotune

tar -zxvf $BASE_DIR/../custom-scripts/packages/xenomai-3.tar.gz -C $BASE_DIR/build
pushd $BASE_DIR/build/xenomai-3
scripts/bootstrap
export CFLAGS="-march=armv7-a -mfloat-abi=hard -mfpu=neon -ffast-math"
export LDFLAGS="-march=armv7-a -mfloat-abi=hard -mfpu=neon -ffast-math"
export DESTDIR=$BASE_DIR/target
mkdir -p build
pushd build
../configure --enable-smp --host=arm-linux-gnueabihf --with-core=cobalt
make && make install
popd
popd

make -C $BASE_DIR/../custom-scripts/gpio_test/
cp  $BASE_DIR/../custom-scripts/gpio_test/xen-gpio $BASE_DIR/target/usr/xenomai/bin

make -C $BASE_DIR/../custom-scripts/spi_test
cp  $BASE_DIR/../custom-scripts/spi_test/xen_spi $BASE_DIR/target/usr/xenomai/bin


#FIXME: This entry will repeat at each recompilation.
echo -e >> $BASE_DIR/target/etc/profile
echo -e "export LD_LIBRARY_PATH=/usr/xenomai/lib" >> $BASE_DIR/target/etc/profile




