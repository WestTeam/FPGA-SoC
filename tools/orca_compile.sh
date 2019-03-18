#!/bin/sh

#/opt/altera16.1_lite/nios2eds/nios2_command_shell.sh

export LM_LICENSE_FILE=

cd ../sw/

cd low_level
make clean
make

cd ../lidar
make clean
make

cd ../pid
make clean
make

cd ../odometry
make clean
make

cd ../trajectory
make clean
make

cd ../../orca/tools

rm low_level.elf.qex
./elf2hex.sh ../../sw/low_level/low_level.elf
cp low_level.elf.qex ../../quartus/system_ll.hex

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


#quartus_cdb --update_mif ../..//quartus/HPSFPGA.qpf
#quartus_asm ../..//quartus/HPSFPGA.qpf
#quartus_cpf -c -o bitstream_compression=on ../..//quartus/outputs/HPSFPGA.sof ../..//quartus/outputs/soc_system.rbf

quartus_cdb --rev=DE10 --update_mif ../..//quartus/HPSFPGA.qpf
quartus_asm --rev=DE10 ../..//quartus/HPSFPGA.qpf
quartus_cpf -c -o bitstream_compression=on ../..//quartus/outputs/DE10.sof ../..//quartus/outputs/soc_system_de10.rbf


