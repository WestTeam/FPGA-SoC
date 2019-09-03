#!/bin/sh

#/opt/altera16.1_lite/nios2eds/nios2_command_shell.sh

echo "Program DE10 board through JTAG (.sof)"

cd "$(dirname "$0")"

killall nios2-terminal

cd ../quartus/
quartus_pgm pgm_de10.cdf

if [ "$#" -eq 1 ]; then
    nios2-terminal -i $1
fi
