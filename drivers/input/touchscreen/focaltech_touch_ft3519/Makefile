# Makefile for the focaltech touchscreen drivers.



obj-$(CONFIG_TOUCHSCREEN_FT3519) += focaltech-ft3519.o
focaltech-ft3519-y := focaltech_core.o \
		focaltech_ex_fun.o \
		focaltech_ex_mode.o \
		focaltech_gesture.o \
		focaltech_esdcheck.o \
		focaltech_point_report_check.o


focaltech-ft3519-y += focaltech_i2c.o





focaltech-ft3519-y += focaltech_flash.o
focaltech-ft3519-y += focaltech_flash/focaltech_upgrade_ft3519.o
