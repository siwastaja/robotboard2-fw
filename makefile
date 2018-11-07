# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded

DEVIP = 192.168.1.6

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

OBJ = stm32init.o main.o flash.o own_std.o tof_muxing.o tof_ctrl.o tof_process.o tof_table.o micronavi.o adcs.o pwrswitch.o charger.o bldc.o imu.o audio.o sbc_comm.o timebase.o backup_ram.o run.o
ASMS = stm32init.s main.s flash.s own_std.s tof_muxing.s tof_ctrl.s tof_process.s tof_table.s micronavi.s adcs.s pwrswitch.s charger.s bldc.s imu.s audio.s sbc_comm.s timebase.s backup_ram.s run.s

CFLAGS = -I. -Os -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m7 -specs=nano.specs -Wall -Winline -fstack-usage -DSTM32H743xx -mfloat-abi=hard -mfpu=fpv5-d16 -fno-strict-aliasing -Wno-discarded-qualifiers

CFLAGS += -DFIRMWARE

CFLAGS += -DSBC_RASPI

ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m7 -mthumb -nostartfiles -mfloat-abi=hard -mfpu=fpv5-d16 -specs=nano.specs

all: main.bin

%.o: %.c
	$(CC) -c $(CFLAGS) $*.c -o $*.o
	$(CC) -MM $(CFLAGS) $*.c > $*.d
	@mv -f $*.d $*.d.tmp
	@sed -e 's|.*:|$*.o:|' < $*.d.tmp > $*.d
	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
	@rm -f $*.d.tmp

main.bin: $(OBJ)
	$(LD) -Tlinker.ld $(LDFLAGS) -o main.elf $^ -lm
	$(OBJCOPY) -Obinary --remove-section=.ARM* --remove-section=*bss* --remove-section=.settings main.elf main_full.bin
	$(SIZE) main.elf

# NOTE: I need a custom bss section in the core-coupled DTCM RAM. This should be a piece of cake, but it isn't.
# Tried to find a way to tell GCC/LD to mark this custom bss section with ALLOC flag only, but it always gets generated with CONTENTS, ALLOC, LOAD, DATA,
# causing the binary file to extend to the address space of the totally wrong (meaningless) "load address", generating 500 megabyte binary.
# Since bss sections have nothing to do in the binary, we can just as well remove them from the output file.
# .settings is also removed - this makes the binary small, and keeps the old settings through reflash process

ff: main.bin
	scp main_full.bin $(DEVIP):~/robotsoft/main_full.bin
	ssh $(DEVIP) ~/robotsoft/spiprog r 10 ~/robotsoft/main_full.bin

fr:
	ssh $(DEVIP) ~/spiprog r

f: main.bin
	scp main_full.bin pulu@$(robot):~/main_full.bin

flash: main.bin
	stm32sprog -b 115200 -vw main_full.bin

stack:
	cat *.su

sections:
	arm-none-eabi-objdump -h main.elf

syms:
	arm-none-eabi-objdump -t main.elf

%.s: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS) $(ASMFLAGS)

asm: $(ASMS)

e:
	gedit --new-window makefile linker.ld misc.h imu_m_compensation.c `echo "$(OBJ)" | sed s/"\.o"/"\.c"/g` `echo "$(OBJ)" | sed s/"\.o"/"\.h"/g` &

s:
	screen /dev/ttyUSB0 460800
