cp arch/arm/boot/zImage ../img/
cp arch/arm/boot/zImage ../willow_android/device/thinkware/willow/
../willow_android/out/host/linux-x86/bin/mkbootimg --kernel arch/arm/boot/zImage --ramdisk ../willow_android/out/target/product/willow/ramdisk.img --cmdline "console=ttySAC2,115200" --base 40008000 --pagesize 4096 -o boot.img
../willow_android/out/host/linux-x86/bin/mkbootimg --kernel arch/arm/boot/zImage --ramdisk ../willow_android/out/target/product/willow/ramdisk-recovery.img --cmdline "console=ttySAC2,115200" --base 40008000 --pagesize 4096 -o recovery.img
cp boot.img ../img/
cp recovery.img ../img/
