#!/usr/bin/env bash
#
# Copyright (C) 2023 Edwiin Kusuma Jaya (ryuzenn)
#
# Simple Local Kernel Build Script
#
# Configured for Redmi Note 8 / ginkgo custom kernel source
#
# Setup build env with akhilnarang/scripts repo
#
# Use this script on root of kernel directory

SECONDS=0 # builtin bash timer
LOCAL_DIR=/home/ryuzenn/
ZIPNAME="RyzenKernel-AOSP-A11-Ginkgo-$(TZ=Asia/Jakarta date +"%Y%m%d-%H%M").zip"
ZIPNAME_KSU="RyzenKernel-AOSP-A11-Ginkgo-KSU-$(TZ=Asia/Jakarta date +"%Y%m%d-%H%M").zip"
TC_DIR="${LOCAL_DIR}toolchain"
CLANG_DIR="${TC_DIR}/clang-rastamod"
GCC_64_DIR="${LOCAL_DIR}toolchain/aarch64-linux-android-4.9"
GCC_32_DIR="${LOCAL_DIR}toolchain/arm-linux-androideabi-4.9"
AK3_DIR="${LOCAL_DIR}AnyKernel3"
DEFCONFIG="vendor/ginkgo-perf_defconfig"

export PATH="$CLANG_DIR/bin:$PATH"
export KBUILD_BUILD_USER="EdwiinKJ"
export KBUILD_BUILD_HOST="RastaMod69"
export LD_LIBRARY_PATH="$CLANG_DIR/lib:$LD_LIBRARY_PATH"
export KBUILD_BUILD_VERSION="1"
export LOCALVERSION

if ! [ -d "${CLANG_DIR}" ]; then
echo "Clang not found! Cloning to ${TC_DIR}..."
if ! git clone --depth=1 -b clang-20.0 https://gitlab.com/kutemeikito/rastamod69-clang ${CLANG_DIR}; then
echo "Cloning failed! Aborting..."
exit 1
fi
fi

if ! [ -d "${GCC_64_DIR}" ]; then
echo "gcc not found! Cloning to ${GCC_64_DIR}..."
if ! git clone --depth=1 -b lineage-19.1 https://github.com/LineageOS/android_prebuilts_gcc_linux-x86_aarch64_aarch64-linux-android-4.9.git ${GCC_64_DIR}; then
echo "Cloning failed! Aborting..."
exit 1
fi
fi

if ! [ -d "${GCC_32_DIR}" ]; then
echo "gcc_32 not found! Cloning to ${GCC_32_DIR}..."
if ! git clone --depth=1 -b lineage-19.1 https://github.com/LineageOS/android_prebuilts_gcc_linux-x86_arm_arm-linux-androideabi-4.9.git ${GCC_32_DIR}; then
echo "Cloning failed! Aborting..."
exit 1
fi
fi

if [[ $1 = "-k" || $1 = "--ksu" ]]; then
	echo -e "\nCleanup KernelSU first on local build\n"
	rm -rf KernelSU drivers/kernelsu
else
	echo -e "\nSet No KernelSU Install, just skip\n"
fi

# Set function for override kernel name and variants
if [[ $1 = "-k" || $1 = "--ksu" ]]; then
echo -e "\nKSU Support, let's Make it On\n"
curl -kLSs "https://raw.githubusercontent.com/kutemeikito/KernelSU/main/kernel/setup.sh" | bash -s main
sed -i 's/CONFIG_KSU=n/CONFIG_KSU=y/g' arch/arm64/configs/vendor/ginkgo-perf_defconfig
sed -i 's/CONFIG_LOCALVERSION="-RyzenKernel"/CONFIG_LOCALVERSION="-RyzenKernel-KSU"/g' arch/arm64/configs/vendor/ginkgo-perf_defconfig
sed -i 's/CONFIG_MSM_CAMERA_BOOTCLOCK_TIMESTAMP=y/#CONFIG_MSM_CAMERA_BOOTCLOCK_TIMESTAMP=is not set/g' arch/arm64/configs/vendor/ginkgo-perf_defconfig
else
echo -e "\nKSU not Support, let's Skip\n"
sed -i 's/CONFIG_MSM_CAMERA_BOOTCLOCK_TIMESTAMP=y/#CONFIG_MSM_CAMERA_BOOTCLOCK_TIMESTAMP=is not set/g' arch/arm64/configs/vendor/ginkgo-perf_defconfig
fi

mkdir -p out
make O=out ARCH=arm64 $DEFCONFIG

echo -e "\nStarting compilation...\n"
make -j$(nproc --all) O=out \
					  ARCH=arm64 \
					  CC=clang \
					  LD=ld.lld \
					  AR=llvm-ar \
					  AS=llvm-as \
					  NM=llvm-nm \
					  OBJCOPY=llvm-objcopy \
					  OBJDUMP=llvm-objdump \
					  STRIP=llvm-strip \
					  CROSS_COMPILE=aarch64-linux-android- \
					  CROSS_COMPILE_COMPAT=arm-linux-gnueabi- \
					  CLANG_TRIPLE=aarch64-linux-gnu- \
					  Image.gz-dtb \
					  dtbo.img

if [ -f "out/arch/arm64/boot/Image.gz-dtb" ] && [ -f "out/arch/arm64/boot/dtbo.img" ]; then
echo -e "\nKernel compiled succesfully! Zipping up...\n"
git restore arch/arm64/configs/vendor/ginkgo-perf_defconfig
if [ -d "$AK3_DIR" ]; then
cp -r $AK3_DIR AnyKernel3
elif ! git clone -q https://github.com/kutemeikito/AnyKernel3; then
echo -e "\nAnyKernel3 repo not found locally and cloning failed! Aborting..."
exit 1
fi
cp out/arch/arm64/boot/Image.gz-dtb AnyKernel3
cp out/arch/arm64/boot/dtbo.img AnyKernel3
rm -f *zip
cd AnyKernel3
git checkout main &> /dev/null
if [[ $1 = "-k" || $1 = "--ksu" ]]; then
zip -r9 "../$ZIPNAME_KSU" * -x '*.git*' README.md *placeholder
else
zip -r9 "../$ZIPNAME" * -x '*.git*' README.md *placeholder
fi
cd ..
rm -rf AnyKernel3
rm -rf out/arch/arm64/boot
echo -e "======================================="
echo -e "░█▀▀█ █──█ ▀▀█ █▀▀ █▀▀▄ "
echo -e "░█▄▄▀ █▄▄█ ▄▀─ █▀▀ █──█ "
echo -e "░█─░█ ▄▄▄█ ▀▀▀ ▀▀▀ ▀──▀ "
echo -e " "
echo -e "░█─▄▀ █▀▀ █▀▀█ █▀▀▄ █▀▀ █── "
echo -e "░█▀▄─ █▀▀ █▄▄▀ █──█ █▀▀ █── "
echo -e "░█─░█ ▀▀▀ ▀─▀▀ ▀──▀ ▀▀▀ ▀▀▀ "
echo -e "======================================="
echo -e "Completed in $((SECONDS / 60)) minute(s) and $((SECONDS % 60)) second(s) !"
if [[ $1 = "-k" || $1 = "--ksu" ]]; then
echo "Zip: $ZIPNAME_KSU"
else
echo "Zip: $ZIPNAME"
fi
else
echo -e "\nCompilation failed!"
exit 1
fi
echo "Move Zip into Home Directory"
mv *.zip ${LOCAL_DIR}
echo -e "======================================="