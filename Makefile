BSP = aarch64-rtems6-xilinx_zynqmp_lp64_qemu
#BSP = arm-rtems6-xilinx_zynq_a9_qemu-default
#TEST = build/$(BSP)/pf01.exe
#TEST = build/$(BSP)/selectpollkqueue01.exe
TEST = build/$(BSP)/telnetd01.exe
#TEST = build/$(BSP)/condvar01.exe
#TEST = build/$(BSP)/unix01.exe
#TEST = build/$(BSP)/syscalls01.exe
#TEST = build/$(BSP)/cdev01.exe
#TEST = build/$(BSP)/media01.exe
#TEST = build/$(BSP)/mutex01.exe

#QEMU = qemu-system-arm -nographic -nic tap,ifname=qtap,script=no,downscript=no -serial null -serial mon:stdio -machine xilinx-zynq-a9 -m 256M
#QEMU = qemu-system-arm -nographic -serial null -serial mon:stdio -machine xilinx-zynq-a9 -m 256M -net tap,ifname=qtap,script=no,downscript=no -net nic,model=cadence_gem,macaddr=0e:b0:ba:5e:ba:12
QEMU = qemu-system-aarch64 -no-reboot -nographic -serial mon:stdio -machine xlnx-zcu102 -m 4096 -nic tap,ifname=qtap,script=no,downscript=no

all:
	./waf -j 1

run: all
	$(QEMU) -kernel $(TEST) -no-reboot

qtap:
	sudo ip tuntap add qtap mode tap user $$(whoami)
	sudo ip addr add 10.0.2.1/24 dev qtap
	sudo ip link set qtap up

qemu:
	$(QEMU) -kernel $(TEST) -s -S

gdb:
	/opt/rtems/6/bin/arm-rtems6-gdb --command=$(HOME)/bin/arm.gdb $(TEST)
