U-Boot for the AAEON iMX8P IoT Gateway

Quick Start
===========
* Build the ARM Trusted firmware binary
* Get ddr and hdmi fimware
* Build U-Boot
* Boot

Get and Build the ARM Trusted firmware
======================================
Get ATF from: https://source.codeaurora.org/external/imx/imx-atf

branch: lf_v2.4
```
$ make PLAT=imx8mp bl31 # without OP-TEE
or
$ make PLAT=imx8mp SPD=opteed bl31 # with OP-TEE

$ cp build/imx8mp/release/bl31.bin $(builddir)
```

Get the ddr and hdmi firmware
=============================
```
$ wget https://www.nxp.com/lgfiles/NMG/MAD/YOCTO/firmware-imx-8.11.bin
$ chmod +x firmware-imx-8.11.bin
$ ./firmware-imx-8.11.bin
$ cp firmware-imx-8.11/firmware/hdmi/cadence/signed_hdmi_imx8m.bin $(builddir)

```

Build U-Boot
============
```
$ export CROSS_COMPILE=<aarch64 toolchain path>
$ make distclean
$ make srg-mx8p_defconfig
$ make all -j8
```

Burn the flash.bin to MicroSD card offset 32KB
==============================================
```
dd if=flash.bin of=/dev/sd[x] bs=1K seek=32 conv=sync
$sudo dd if=u-boot.itb of=/dev/sd[x] bs=1K seek=384 conv=sync
```

Boot
====
Set Boot switch SW5: 0011 to boot from Micro SD.

Get and Build OP-TEE OS
=======================
Get OP-TEE OS from: https://source.codeaurora.org/external/imx/imx-optee-os
branch: lf-5.10.y_1.0.0
```
$make ARCH=arm PLATFORM=imx PLATFORM_FLAVOR=mx8mpevk
$aarch64-linux-gnu-objcopy -O binary tee.elf tee.bin
```
