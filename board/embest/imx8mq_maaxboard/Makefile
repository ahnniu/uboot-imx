#
# Copyright 2017 Freescale Semiconductor
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += imx8mq_maaxboard.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-$(CONFIG_MAAXBOARD_DDR4_HYNIX_2GB) += ddr4_timing_hynix_2gb.o
obj-$(CONFIG_MAAXBOARD_DDR4_HYNIX_4GB) += ddr4_timing_hynix_4gb.o
endif

obj-y += ../../freescale/common/mmc.o
