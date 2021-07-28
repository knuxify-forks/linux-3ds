obj-$(CONFIG_CTR_GPIO)	+= ctr_gpio.o

obj-$(CONFIG_CTR_SPI)	+= ctr_spi.o
obj-$(CONFIG_CTR_TSC)	+= ctr_tsc.o
obj-$(CONFIG_CTR_TSC_TOUCH)	+= tsc/touch.o

obj-$(CONFIG_CTR_I2C)	+= ctr_i2c.o

# interrupt controller and regulator are
# always enabled if the MCU is enabled
obj-$(CONFIG_CTR_MCU)	+= mcu/intc.o
obj-$(CONFIG_CTR_MCU)	+= mcu/regulator.o

obj-$(CONFIG_CTR_MCUACCEL)	+= mcu/accel.o
obj-$(CONFIG_CTR_MCULED)	+= mcu/led.o
obj-$(CONFIG_CTR_MCURTC)	+= mcu/rtc.o
obj-$(CONFIG_CTR_MCUCHARGER)	+= mcu/charger.o

obj-$(CONFIG_CTR_PXI)	+= ctr_pxi_hw.o

# don't compile any of these until PXI is finalized
obj-$(CONFIG_CTR_PRNG)	+= pxi/ctr_prng.o
obj-$(CONFIG_CTR_SDHC_MMIO)	+= ctr_sdhc_mmio.o
# obj-$(CONFIG_CTR_SDHC_PXI)	+= ctr_sdhc_pxi.o
