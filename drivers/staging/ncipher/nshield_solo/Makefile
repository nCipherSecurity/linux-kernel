ifneq ($(KERNELRELEASE),)
include Kbuild
else
obj-$(CONFIG_HSM_NSHIELD_SOLO) := nshield_solo.o
nshield_solo-y := osif.o hostif.o fsl.o i21555.o drvlist.o
endif
