#!/bin/sh

echo "Software compilation (ORCA)"

cd ../sw/

cd low_level
make clean
make

status=$?
[ $status -eq 0 ] || exit 1

cd ../imu
make clean
make

status=$?
[ $status -eq 0 ] || exit 1

cd ../lidar
make clean
make

status=$?
[ $status -eq 0 ] || exit 1

cd ../pid
make clean
make

status=$?
[ $status -eq 0 ] || exit 1

cd ../odometry
make clean
make

status=$?
[ $status -eq 0 ] || exit 1

cd ../trajectory
make clean
make

status=$?
[ $status -eq 0 ] || exit 1

cd ../../orca/tools

rm low_level.elf.qex
./elf2hex.sh ../../sw/low_level/low_level.elf
cp low_level.elf.qex ../../quartus/system_ll.hex

rm imu.elf.qex
./elf2hex.sh ../../sw/imu/imu.elf
cp imu.elf.qex ../../quartus/imu.hex

rm lidar.elf.qex
./elf2hex.sh ../../sw/lidar/lidar.elf
cp lidar.elf.qex ../../quartus/lidar.hex

rm pid.elf.qex
./elf2hex.sh ../../sw/pid/pid.elf
cp pid.elf.qex ../../quartus/pid.hex

rm odometry.elf.qex
./elf2hex.sh ../../sw/odometry/odometry.elf
cp odometry.elf.qex ../../quartus/odometry.hex

rm trajectory.elf.qex
./elf2hex.sh ../../sw/trajectory/trajectory.elf
cp trajectory.elf.qex ../../quartus/trajectory.hex

