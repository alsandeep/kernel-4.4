#
# Copyright (c) 2014 Actions Semiconductor Co., Ltd.
#
# Author: Yixun Lan <yixun.lan@gmail.com>

# Licensed under GPLv2

ccflags-$(CONFIG_ARCH_MULTIPLATFORM) := -I$(srctree)/$(src)/include

obj-$(CONFIG_ARCH_OWL) 	+= owl.o
obj-$(CONFIG_ARCH_OWL) 	+= cpu-owl.o s500-clocktree.o  clocktree-owl.o
obj-$(CONFIG_ARCH_OWL)  += bootdev-owl.o powergate-owl.o 
