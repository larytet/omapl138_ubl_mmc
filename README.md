omapl138_ubl_mmc
================

Set of tools to program MMC and  boot from MMC for  OMAPL138


How to compile
================

On 64 bits machine allow to execute 32 bits applications
apt-get install ia32-libs uboot-mkimage

Download the SDK (Low Cost - LCDK) from 
http://downloads.ti.com/dsps/dsps_public_sw/c6000/web/omapl138_lcdk_sdk/latest/exports/ti-sdk-omapl138-lcdk-01.00.00.bz2

Open the archive
tar xjvf ti-sdk-omapl138-lcdk-01.00.00.bz2

Install ti_cgt_c6000_7.3.3_setup_linux_x86.bin, bios_setuplinux_6_33_02_31.bin, ipc_setuplinux_1_24_02_27.bin, xdctools_setuplinux_3_23_01_43.bin from dsp-tools - use folder 'ti' your home directory. Folder ~ti shall look like this
  $ ls ~/ti -w 1
  bios_6_33_02_31
  ipc_1_24_02_27
  linux-devkit
  syslink_2_10_03_20
  TI_CGT_C6000_7.3.3
  xdctools_3_23_01_43
  
Copy folders linux-devkit and syslink to the ~/ti

Compile
make -C audio/ubl/mk/ all

In the audio/ubl/mk find file ubl.out


Load UBL to OMAP
================

Via UART 

/omapl138_boot_loader.py -d /dev/ttyUSB0 -l "./mk/ubl.out.bin  0x80000000 0x80000000"

Make sure that the script prints "Load done" in the end of the execution

Program MMC
================

Switch to the UART boot mode, connect serial terminal (115200, no parity, one stop bit)
On the evaluation board set swicthes 
1   2   3   4   5   6   7   8
Off On  Off On  Off Off Off Off

Power cycle the board and see "BOOTME" in the serial terminal
Close the terminal
Load UBL code to the RAM (alternatively use JTAG and CCS to load and run the ubl.out)
Program UBL code on the MMC BLOCK 0
./omapl138_boot_loader.py  --mmc './mk/ubl.out.bin.ais 0 0x80000000'

Switch to the MMC boot mode, connect serial terminal (115200, no parity, one stop bit)
On the evaluation board set switches 
1   2   3   4   5   6   7   8
Off On  On  On  Off Off Off Off

Power cycle the board and see "UBL Wait for user command boot/prog:" in the serial terminal

Close the terminal

Program u-boot to the MMC BLOCK 50

./omapl138_boot_loader.py  --mmc 'u-boot.bin.ais 50 0xC1080000'
