cp arch/arm/boot/zImage ../img/
cp arch/arm/boot/zImage ../willow_android/device/thinkware/willow/
../willow_android/out/host/linux-x86/bin/mkbootimg --kernel arch/arm/boot/zImage --ramdisk ../willow_android/out/target/product/willow/ramdisk.img --cmdline "console=ttySAC2,115200" --base 40008000 --pagesize 4096 -o ../willow_android/out/target/product/willow/boot.img
../willow_android/out/host/linux-x86/bin/mkbootimg --kernel arch/arm/boot/zImage --ramdisk ../willow_android/out/target/product/willow/ramdisk-recovery.img --cmdline "console=ttySAC2,115200" --base 40008000 --pagesize 4096 -o ../willow_android/out/target/product/willow/recovery.img
cp drivers/net/wireless/bcmdhd/bcmdhd.ko ../willow_android/out/target/product/willow/system/lib/modules/
cp drivers/net/wireless/bcmdhd/bcmdhd.ko ../willow_android/device/thinkware/willow/

# JB
cp arch/arm/boot/zImage ../willow_jb_android/device/thinkware/willow/
../willow_jb_android/out/host/linux-x86/bin/mkbootimg --kernel arch/arm/boot/zImage --ramdisk ../willow_jb_android/out/target/product/willow/ramdisk.img --cmdline "console=ttySAC2,115200" --base 40008000 --pagesize 4096 -o ../willow_jb_android/out/target/product/willow/boot.img
../willow_jb_android/out/host/linux-x86/bin/mkbootimg --kernel arch/arm/boot/zImage --ramdisk ../willow_jb_android/out/target/product/willow/ramdisk-recovery.img --cmdline "console=ttySAC2,115200" --base 40008000 --pagesize 4096 -o ../willow_jb_android/out/target/product/willow/recovery.img
cp drivers/net/wireless/bcmdhd/bcmdhd.ko ../willow_jb_android/out/target/product/willow/system/lib/modules/
cp drivers/net/wireless/bcmdhd/bcmdhd.ko ../willow_jb_android/device/thinkware/willow/

