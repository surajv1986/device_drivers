obj-m := i2c_adxl_drv.o
all:
	make -C /lib/modules/4.19.95-v7+/build M=$(PWD) modules
clean:
	make -C /lib/modules/4.19.95-v7+/build M=$(PWD) clean
