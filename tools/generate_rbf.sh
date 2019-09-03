#!/bin/sh

echo "DE10 .rbf file generation"

quartus_cpf -c -o bitstream_compression=on ../..//quartus/outputs/DE10.sof ../..//quartus/outputs/soc_system_de10.rbf


