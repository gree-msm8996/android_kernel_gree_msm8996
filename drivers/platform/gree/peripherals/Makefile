# Gree G0245D private peripherals.
#
# Every driver here is a NEW file ported from the hardware-verified 3.18
# reconstruction.  No upstream driver (in particular qcacld-2.0 WLAN and
# MDSS/DSI panel) is modified.  Symbols are defined in ../Kconfig.

obj-$(CONFIG_G0245D_TOUCHSCREEN_FT8716)   += focaltech_core.o
obj-$(CONFIG_G0245D_FINGERPRINT_GF816M)   += gf_spi.o
obj-$(CONFIG_G0245D_TUSB320)               += tusb320.o
obj-$(CONFIG_G0245D_RAON_REMOTE_CTRL)      += remote_ctrl.o
obj-$(CONFIG_G0245D_LDO_MOTOR)             += ldo_motor.o
obj-$(CONFIG_G0245D_ZIGBEE)                += zigbee.o

# NXP TFA98xx smart amplifier (OnePlus OSS QC8996_N lineage).
snd-soc-g0245d-tfa-objs := tfa98xx.o tfa_dsp.o tfa_container.o
obj-$(CONFIG_G0245D_TFA98XX)               += snd-soc-g0245d-tfa.o
