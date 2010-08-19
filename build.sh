#!/bin/bash -x

set -e

SCRIPT_PATH=`readlink -f $0`
export REPO_DIR=`dirname "${SCRIPT_PATH}"`
echo "Repository path: ${REPO_DIR}"
export KERNEL_DIR="$REPO_DIR/linux"
KERNEL_IMAGE="$KERNEL_DIR/arch/arm/boot/zImage"
ROOTFS_DIR="$REPO_DIR/rootfs_patch"

# Configure options.
export ONYX_SDK_ROOT=/opt/onyx/arm
export PATH=$PATH:$ONYX_SDK_ROOT/bin:/opt/freescale/usr/local/gcc-4.1.2-glibc-2.5-nptl-3/arm-none-linux-gnueabi/bin
HOST=arm-linux
BUILD=i686

GUESSED_BRANCH=`cd "${REPO_DIR}" && git branch | grep '^\*' | cut -d ' ' -f 2`
GIT_BRANCH=${GIT_BRANCH-master}
BRANCH=${GIT_BRANCH-${GUESSED_BRANCH}}
echo "Building branch: $BRANCH"

if [ ! -d "$REPO_DIR" ]; then
    echo "$REPO_DIR does not exist."
    exit 1
fi

kernel_config=onyx_defconfig
initramfs_config=onyx_initramfs_defconfig

cd "$REPO_DIR"/onyx/initramfs && make

rm -rf "$ROOTFS_DIR"
mkdir "$ROOTFS_DIR"

# Build kernel modules, and put them into rootfs
cd $KERNEL_DIR
ARCH=arm CROSS_COMPILE=arm-linux- make $kernel_config
ARCH=arm CROSS_COMPILE=arm-linux- make modules
ARCH=arm CROSS_COMPILE=arm-linux- make modules_install INSTALL_MOD_PATH=$ROOTFS_DIR

# Build zImage (with and without initramfs)
cd $KERNEL_DIR
ARCH=arm CROSS_COMPILE=arm-linux- make $initramfs_config
ARCH=arm CROSS_COMPILE=arm-linux- make zImage
cp "$KERNEL_IMAGE" "$REPO_DIR/zImage-initramfs"
ARCH=arm CROSS_COMPILE=arm-linux- make $kernel_config
ARCH=arm CROSS_COMPILE=arm-linux- make zImage
cp "$KERNEL_IMAGE" "$REPO_DIR/"
