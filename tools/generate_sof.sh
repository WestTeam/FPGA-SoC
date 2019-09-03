#!/bin/sh

echo "DE10 .sof file generation"

quartus_cdb --rev=DE10 --update_mif ../quartus/HPSFPGA.qpf

status=$?
[ $status -eq 0 ] || exit 1

quartus_asm --rev=DE10 ../quartus/HPSFPGA.qpf

status=$?
[ $status -eq 0 ] || exit 1
