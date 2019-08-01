ifneq ($(KERNELRELEASE),)
include Kbuild
else
obj-$(CONFIG_HSM_NSHIELD_SOLO) := nfp.o
nfp-y := osif.o hostif.o fsl.o i21555.o drvlist.o
endif
