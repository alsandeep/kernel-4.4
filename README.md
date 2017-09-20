# kernel-4.4
Kernel 4.4

Steps:
1) dowloaded from linux distro kernel-4.4

   v4.4 commit
   commit afd2ff9b7e1b367172f18ba7f693dfb62bdcb2dc
   Author: Linus Torvalds <torvalds@linux-foundation.org>
   Date:   Sun Jan 10 15:01:32 2016 -0800
   -----------

2) apply patch from http://dev.linux-xapple.org/~dennis/4.4.x-porting/
   kernel_patches_v4.4.tar.bz2

3) Modify owl Make file to add LOADADDR=0x00008000 to build kernel
   kernel: $(K_BLD_CONFIG)
        $(Q)mkdir -p $(KERNEL_OUT_DIR)
        $(Q)$(MAKE) -C $(KERNEL_SRC) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) O=$(KERNEL_OUT_DIR) dtbs
        $(Q)$(MAKE) -C $(KERNEL_SRC) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) O=$(KERNEL_OUT_DIR) -j$(CPUS) uImage  LOADADDR=0x00008000

4) Modify kernel config arch/arm/configs/s500_defconfig
 	and  change 
 	CONFIG_DEBUG_UART_PHYS=0xb0124000
 	CONFIG_DEBUG_UART_VIRT=0xf8124000

5) Modify uart for sparky as ttyS3
 	arch/arm/boot/dts/s500-evb.dts
	default tested on s500 bubblegum board the serial console and lowlevel uart debug output is using UART2
---------------Board boots with UART3 Enabled----------------------
6) Download basic initramfs from https://github.com/xapp-le/linux
	use this slim initramfs instead of heavy one in SDK
	change image name as ramdisk.img

7) Modify Comand line bootargs and add ttyS3 at end
before 		Kernel command line: console=ttyS3 earlyprintk clk_ignore_unused selinux=0 root=/dev/mmcblk0p2 console=tty0 real_rootflags=rw boot_dev=sd0
after changes Kernel command line: console=ttyS3 earlyprintk clk_ignore_unused selinux=0 root=/dev/mmcblk0p2 console=ttyS3 real_rootflags=rw boot_dev=sd0
