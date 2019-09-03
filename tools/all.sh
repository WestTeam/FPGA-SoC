#!/bin/sh

./sw_compile.sh

status=$?
[ $status -eq 0 ] || exit 1

./generate_sof.sh

status=$?
[ $status -eq 0 ] || exit 1

./pgm.sh

if [ "$#" -eq 1 ]; then
    nios2-terminal -i $1
    exit 0
fi

./generate_rbf.sh
