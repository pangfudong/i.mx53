#!/bin/bash

set -e

kernel_config=onyx_defconfig
initramfs_config=onyx_initramfs_defconfig
aes_password="a8wZ49?b"
init_config=default

while [ $# -gt 0 ]
do
    case $1
        in
        --config-prefix)
            CONFIG_PREFIX=$2
            kernel_config="${CONFIG_PREFIX}_defconfig"
            initramfs_config="${CONFIG_PREFIX}_initramfs_defconfig"
            shift 2
            ;;

        --aes-password)
            aes_password=$2
            shift 2
            ;;

        --init-config)
            init_config=$2
            shift 2
            ;;

        *)
            echo "Invalid arguement: $1"
            echo "Valid arguments are"
            echo "--config-prefix prefix: Prefix of the kernel config files."
            echo "--aes-password password: The password used to encrypt update packages."
            echo "--init-config name: The configuration to prepend to the init script."
            exit 1
            ;;
    esac
done

echo "Using kernel config: $kernel_config"
echo "Using initramfs config: $initramfs_config"

SCRIPT_PATH=`readlink -f $0`
REPO_DIR=`dirname "${SCRIPT_PATH}"`
echo "Repository path: ${REPO_DIR}"
export KERNEL_DIR="$REPO_DIR/linux"
KERNEL_IMAGE="$KERNEL_DIR/arch/arm/boot/zImage"
ROOTFS_DIR="$REPO_DIR/rootfs_patch"
INIT_SCRIPT_PATH="$REPO_DIR/onyx/initramfs/init"
INIT_CONFIG_DIR="$REPO_DIR/onyx/initramfs.d"
INIT_SCRIPT_MAIN_PATH="$INIT_CONFIG_DIR/init.in.sh"

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

echo "Generating ${INIT_SCRIPT_PATH} using configuration ${init_config}"
echo "#!/bin/sh" > "${INIT_SCRIPT_PATH}"
cat "${INIT_CONFIG_DIR}/${init_config}.conf.sh" >> "${INIT_SCRIPT_PATH}"
cat "${INIT_SCRIPT_MAIN_PATH}" >> "${INIT_SCRIPT_PATH}"

echo "Updating password in ${INIT_SCRIPT_PATH}"
sed -i "s/__AES_PASSWORD__/${aes_password}/" "${INIT_SCRIPT_PATH}"

sed "s|REPO_DIR|$REPO_DIR|" "$KERNEL_DIR/arch/arm/configs/${initramfs_config}.in" \
    > "$KERNEL_DIR/arch/arm/configs/${initramfs_config}"

cd "$REPO_DIR"/linux/usr && make gen_init_cpio
cd "$REPO_DIR"/onyx/initramfs && make clean
cd "$REPO_DIR"/onyx/initramfs && make

rm -rf "$ROOTFS_DIR"
mkdir "$ROOTFS_DIR"

# Build zImage (with and without initramfs)
cd "$KERNEL_DIR"
ARCH=arm CROSS_COMPILE=arm-linux- make $initramfs_config
ARCH=arm CROSS_COMPILE=arm-linux- make zImage
cp "$KERNEL_IMAGE" "$REPO_DIR/zImage-initramfs"
ARCH=arm CROSS_COMPILE=arm-linux- make $kernel_config
ARCH=arm CROSS_COMPILE=arm-linux- make zImage
cp "$KERNEL_IMAGE" "$REPO_DIR/"

# Build kernel modules, and put them into rootfs
ARCH=arm CROSS_COMPILE=arm-linux- make modules
ARCH=arm CROSS_COMPILE=arm-linux- make modules_install INSTALL_MOD_PATH=$ROOTFS_DIR
