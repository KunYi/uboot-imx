setenv bootargs console=${console} root=${mmcroot};

for boot_target in ${boot_targets};
do
        if test "${boot_target}" = "mmc1" ; then
                if ext4load mmc 1:${bootpart} ${kernel_addr_r} ${image}; then
                        if ext4load mmc 1:${bootpart} ${fdt_addr} ${fdt_file}; then
                                echo Load image and .dtb from SD card(mmc1);
                                booti ${kernel_addr_r} - ${fdt_addr};
                                exit;
                        fi
                fi
        fi

        if test "${boot_target}" = "mmc2" ; then
                if ext4load mmc 2:${bootpart} ${kernel_addr_r} ${image}; then
                        if ext4load mmc 2:${bootpart} ${fdt_addr} ${fdt_file}; then
                                echo Load image and .dtb from eMMC(mmc2);
                                booti ${kernel_addr_r} - ${fdt_addr};
                                exit;
                        fi
                fi
        fi

done
