obj-m := memdev.o

KDIR := /home/orange/Desktop/linux-3.0.8

all:
	make -C $(KDIR) M=$(PWD) modules CROSS_COMPILE=arm-linux- ARCH=arm

clean:
	rm -f *.o *.ko *.order *.symvers *.mod.*
