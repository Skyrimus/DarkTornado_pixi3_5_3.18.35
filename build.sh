#!/bin/bash
export KBUILD_BUILD_USER=skyrimus
export KBUILD_BUILD_HOST=4pda
export CROSS_COMPILE=/root/arm-eabi-4.8/bin/arm-eabi-
export USE_CCACHE=1
export ARCH=arm ARCH_MTK_PLATFORM=mt6580
export TARGET=out
make O=$TARGET ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE pixi3_5_defconfig
make O=$TARGET ARCH=$ARCH CROSS_COMPILE=$CROSS_COMPILE -j2

