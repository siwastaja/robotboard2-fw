# This makefile is made to work with the toolchain downloadable at https://launchpad.net/gcc-arm-embedded
# More exactly, https://launchpad.net/~team-gcc-arm-embedded/+archive/ubuntu/ppa


DEVUSR = pulu
#DEVIP = 10.3.0.6
#DEVIP = 192.168.43.59
DEVIP = 10.42.0.104
#DEVIP = 192.168.1.6

CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
SIZE = arm-none-eabi-size
OBJCOPY = arm-none-eabi-objcopy

CFLAGS = -I. -fno-common -ffunction-sections -ffreestanding -fno-builtin -mthumb -mcpu=cortex-m7 -specs=nano.specs -Wall -Winline -fstack-usage -DSTM32H743xx -mfloat-abi=hard -mfpu=fpv5-d16 -fno-strict-aliasing -Wno-discarded-qualifiers

CFLAGS += -DFIRMWARE

# PCB revision. Valid values:
# -DREV2A (the single existing prototype)
# -DREV2B (first 50pcs production batch, quite a few IO remappings compared to REV2A)
CFLAGS += -DREV2B


# Hard-compiled application(s):
#CFLAGS += -DEXT_VACUUM
#CFLAGS += -DVACUUM_REV2

# Battery size
CFLAGS += -DBATTERY_SIZE_M

# Charger contacts on back?
#CFLAGS += -DCONTACTS_ON_BACK

#Standard compilation
OBJ_OS = stm32init.o main.o flash.o own_std.o tof_muxing.o tof_ctrl.o  tof_table.o sin_lut.o micronavi.o adcs.o pwrswitch.o charger.o bldc.o imu.o drive.o audio.o sbc_comm.o timebase.o backup_ram.o run.o
#ext_vacuum_boost.o
OBJ_O3 = tof_process.o
ASMS = stm32init.s main.s flash.s own_std.s tof_muxing.s tof_ctrl.s tof_process.s tof_table.s micronavi.s adcs.s pwrswitch.s charger.s bldc.s imu.s audio.s sbc_comm.s timebase.s backup_ram.s run.s


#Calibration: box
#OBJ_OS = stm32init.o main.o flash.o own_std.o tof_muxing.o tof_ctrl.o sin_lut.o adcs.o pwrswitch.o charger.o bldc.o imu.o audio.o sbc_comm.o timebase.o backup_ram.o ../robotboard2-fw-calibrator/run_box.o ../robotboard2-fw-calibrator/tof_calibrator.o
#CFLAGS += -DCALIBRATOR
#CFLAGS += -DCALIBRATOR_BOX

#Calibration: wall
#OBJ_OS = stm32init.o main.o flash.o own_std.o tof_muxing.o tof_ctrl.o sin_lut.o adcs.o pwrswitch.o charger.o bldc.o imu.o audio.o sbc_comm.o timebase.o backup_ram.o ../robotboard2-fw-calibrator/run_wall.o ../robotboard2-fw-calibrator/tof_calibrator.o
#CFLAGS += -DCALIBRATOR
#CFLAGS += -DCALIBRATOR_WALL




CFLAGS += -DSBC_RASPI

ASMFLAGS = -S -fverbose-asm
LDFLAGS = -mcpu=cortex-m7 -mthumb -nostartfiles -mfloat-abi=hard -mfpu=fpv5-d16 -specs=nano.specs

all: main.bin

$(OBJ_OS): %.o: %.c
	$(CC) -c -Os $(CFLAGS) $< -o $@

$(OBJ_O3): %.o: %.c
	$(CC) -c -O3 $(CFLAGS) $< -o $@

#%.o: %.c
#	$(CC) -c $(CFLAGS) $*.c -o $*.o
#	$(CC) -MM $(CFLAGS) $*.c > $*.d
#	@mv -f $*.d $*.d.tmp
#	@sed -e 's|.*:|$*.o:|' < $*.d.tmp > $*.d
#	@sed -e 's/.*://' -e 's/\\$$//' < $*.d.tmp | fmt -1 | \
#	  sed -e 's/^ *//' -e 's/$$/:/' >> $*.d
#	@rm -f $*.d.tmp

main.bin: $(OBJ_OS) $(OBJ_O3)
	$(LD) -Tlinker.ld $(LDFLAGS) -o main.elf $^ -lm
	$(OBJCOPY) -Obinary --remove-section=.ARM* --remove-section=*bss* --remove-section=.settings main.elf main_full.bin
	$(SIZE) main.elf

# NOTE: I need a custom bss section in the core-coupled DTCM RAM. This should be a piece of cake, but it isn't.
# Tried to find a way to tell GCC/LD to mark this custom bss section with ALLOC flag only, but it always gets generated with CONTENTS, ALLOC, LOAD, DATA,
# causing the binary file to extend to the address space of the totally wrong (meaningless) "load address", generating 500 megabyte binary.
# Since bss sections have nothing to do in the binary, we can just as well remove them from the output file.
# .settings is also removed - this makes the binary small, and keeps the old settings through reflash process
ff: main.bin
	scp main_full.bin $(DEVUSR)@$(DEVIP):/home/$(DEVUSR)/robotsoft/main_full.bin
	ssh $(DEVUSR)@$(DEVIP) /home/$(DEVUSR)/robotsoft/spiprog r 10 /home/$(DEVUSR)/robotsoft/main_full.bin

fr:
	ssh $(DEVUSR)@$(DEVIP) /home/$(DEVUSR)/robotsoft/spiprog r

flash: main.bin
	stm32flash -b 115200 -w main_full.bin -v /dev/ttyUSB0

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
	gedit --new-window makefile linker.ld misc.h imu_m_compensation.c `echo "$(OBJ_OS) $(OBJ_O3)" | sed s/"\.o"/"\.c"/g` `echo "$(OBJ_OS) $(OBJ_O3)" | sed s/"\.o"/"\.h"/g` &

s:
	screen /dev/ttyUSB0 460800

clean:
	rm *.o
	rm ../robotboard2-fw-calibrator/*.o

