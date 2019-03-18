all:

INC=-I./ -I../tools/ -I../cvra-modules/modules/quadramp/ -I../cvra-modules/modules/trajectory_manager/ -I../cvra-modules/modules/position_manager/ -I../cvra-modules/modules/control_system_manager/ -I../cvra-modules/modules/math/vect2/ -I../cvra-modules/modules/math/geometry/ -I../cvra-modules/modules/robot_system/


AS_SRCS=crt.S

TARGETS=trajectory
TARGET_C_SRCS=$(addsuffix .c,$(TARGETS))
TARGET_C_OBJS=$(addprefix obj/,$(addsuffix .c.o,$(TARGETS)))
C_OBJ_FILES=$(addprefix obj/,$(addsuffix .c.o, ../tools/tools ../cvra-modules/modules/quadramp/quadramp ../cvra-modules/modules/position_manager/2wheels/position_manager ../cvra-modules/modules/control_system_manager/control_system_manager ../cvra-modules/modules/robot_system/2wheels/robot_system ../cvra-modules/modules/math/vect2/vect2 ../cvra-modules/modules/trajectory_manager/2wheels/trajectory_manager ../cvra-modules/modules/trajectory_manager/2wheels/trajectory_manager_core ../cvra-modules/modules/trajectory_manager/2wheels/trajectory_manager_utils  $(C_SRCS)))
S_OBJ_FILES=$(addprefix obj/,$(addsuffix .o, $(AS_SRCS)))

CROSS_COMPILE=riscv32-unknown-elf-
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy
OBJDUMP=$(CROSS_COMPILE)objdump
ARCH=RV32IMXmxp
CFLAGS= -m32 -Os -fno-delete-null-pointer-checks -fdata-sections -ffunction-sections -g -march=$(ARCH) $(INC) -MD -Wall -std=gnu99 -msoft-float -mmuldiv
LD_FLAGS= -m32 -Wl,--gc-sections -Wl,--print-gc-sections -march=$(ARCH) -static -nostartfiles

obj/:
	mkdir -p obj/
	mkdir -p obj/../tools/
	mkdir -p obj/../cvra-modules/modules/quadramp/
	mkdir -p obj/../cvra-modules/modules/math/vect2/
	mkdir -p obj/../cvra-modules/modules/trajectory_manager/2wheels/
	mkdir -p obj/../cvra-modules/modules/position_manager/2wheels/
	mkdir -p obj/../cvra-modules/modules/control_system_manager/
	mkdir -p obj/../cvra-modules/modules/robot_system/2wheels/

$(TARGET_C_OBJS) $(C_OBJ_FILES): obj/%.c.o : %.c | obj/
	$(CC) $(CFLAGS) -c $< -o $@

$(S_OBJ_FILES): obj/%.S.o : %.S | obj/
	$(CC) $(CFLAGS) -c $< -o $@

LD_SCRIPT=link.ld
LIBC=
TARGET_ELFS=$(addsuffix .elf,$(TARGETS))

$(TARGET_ELFS):%.elf:obj/%.c.o $(C_OBJ_FILES) $(S_OBJ_FILES) $(LD_SCRIPT)
	$(CC) -T $(LD_SCRIPT) obj/$*.c.o $(S_OBJ_FILES) $(C_OBJ_FILES) -o $@ $(LD_FLAGS)
	$(OBJDUMP) -D $@ > $*.dump

.phony:all clean
ifndef FORMAT
FORMAT=elf
endif
all: $(TARGET_ELFS)

TARGET_OBJ_DUMPS=$(addsuffix .dump, $(TARGETS))
TARGET_BINS=$(addsuffix .bin, $(TARGETS))
TARGET_HEXS=$(addsuffix .hex, $(TARGETS))
TARGET_MIFS=$(addsuffix .mif, $(TARGETS))
TARGET_MEMS=$(addsuffix .mem, $(TARGETS))
clean:
	rm -rf obj tools cvra-modules $(TARGET_ELFS) $(TARGET_OBJ_DUMPS) $(TARGET_BINS) $(TARGET_HEXS) $(TARGET_HEXS) $(TARGET_MEMS)

-include $(wildcard obj/*.d)
