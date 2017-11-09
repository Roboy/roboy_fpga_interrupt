KERNEL_LOCATION=linux-socfpga
ARMMAKE=make ARCH=arm SUBARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

obj-m := fpga_uinput.o hello_world.o

fpga_uinput.ko: fpga_uinput.c hello_world.c
	$(ARMMAKE) -C $(KERNEL_LOCATION) M=$(PWD) modules 

clean:
	rm -f *.ko *.o *.mod.c *.symvers *.order
