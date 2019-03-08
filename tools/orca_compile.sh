#!/bin/sh

#/opt/altera16.1_lite/nios2eds/nios2_command_shell.sh

export LM_LICENSE_FILE=


cd ../orca/software/generic
make clean
make

cd ../low_level
make clean
make


cd ../../tools/
rm orca.elf.qex
./elf2hex.sh ../software/generic/orca.elf
cp orca.elf.qex ../../quartus/system.hex

./elf2hex.sh ../software/low_level/orca.elf
cp orca.elf.qex ../../quartus/system_ll.hex

#quartus_cdb --update_mif ../..//quartus/HPSFPGA.qpf
#quartus_asm ../..//quartus/HPSFPGA.qpf
#quartus_cpf -c -o bitstream_compression=on ../..//quartus/outputs/HPSFPGA.sof ../..//quartus/outputs/soc_system.rbf

quartus_cdb --rev=DE10 --update_mif ../..//quartus/HPSFPGA.qpf
quartus_asm --rev=DE10 ../..//quartus/HPSFPGA.qpf
quartus_cpf -c -o bitstream_compression=on ../..//quartus/outputs/DE10.sof ../..//quartus/outputs/soc_system_de10.rbf


