#!/bin/sh

make mrproper
make imx5_android_defconfig
make uImage -j8
