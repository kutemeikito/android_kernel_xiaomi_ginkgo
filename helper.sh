#!/bin/bash

DATE=$(date +"%Y%m%d-%H%M")
chat_id=-1001287929514
bot_token=1678018441:AAGMshHVmiYQnl5XyRvm6vc2HM2L2gmxsHE
make_build(){
	print "Make kernel dtb..." green
	if [ $TOOLCHAIN == clang ]; then
		export KBUILD_COMPILER_STRING=$(clang/bin/clang --version | head -n 1 | perl -pe 's/\(http.*?\)//gs' | sed -e 's/  */ /g' -e 's/[[:space:]]*$//')
		PATH="$(pwd)/clang/bin:${PATH}"
		make O=out ARCH=arm64 $1
		make -j$(nproc --all) O=out \
				ARCH=arm64 \
				CC=clang \
				AR=llvm-ar \
				NM=llvm-nm \
				OBJCOPY=llvm-objcopy \
				OBJDUMP=llvm-objdump \
				STRIP=llvm-strip \
				CROSS_COMPILE=aarch64-linux-gnu- \
				CROSS_COMPILE_ARM32=arm-linux-gnueabi-
	else
		export CROSS_COMPILE=$(pwd)/gcc64/bin/aarch64-elf-
		export CROSS_COMPILE_ARM32=$(pwd)/gcc32/bin/arm-eabi-
		make O=out ARCH=arm64 $1
		make -j$(nproc --all) O=out ARCH=arm64
	fi
}

make_clean(){
	rm -rf out
}

send_msg(){
	print "${1}" green
	curl -s -X POST https://api.telegram.org/bot"${bot_token}"/sendMessage \
		-d parse_mode="Markdown" \
		-d chat_id="${chat_id}" \
		-d text="${1}"
}

send_file(){
	print "Sending file..." green
	curl -F chat_id="${chat_id}" \
		-F caption="Build succesfully! | DEVICE: ${type} | SHA1 : $(sha1sum ${file} | awk '{ print $1 }')" \
		-F document=@"${1}" \
		 https://api.telegram.org/bot"${bot_token}"/sendDocument
}

mkzip(){
	print "Generate zip file..." green

	if [ -f $(pwd)/out/arch/arm64/boot/Image.gz-dtb ]; then
		cp $(pwd)/out/arch/arm64/boot/Image.gz-dtb  anykernel
		cd anykernel
		zip -r9 ../$1 * -x .git README.md *placeholder
		cd ..
		send_file $1
	else
		send_msg "Build error !"
		print "Build error !" red
		exit 1
	fi
}

print(){
	echo ""
	case ${2} in
		"red")
		echo -e "\033[31m $1 \033[0m";;
		"green")
		echo -e "\033[32m $1 \033[0m";;
		*)
		echo $1
		;;
	esac
}

gen_toolchain(){
	print "Cloning toolchain files..." green
	if [ $TOOLCHAIN == "clang" ]
	then
		git clone --depth=1 https://github.com/kdrag0n/proton-clang clang
	else
		git clone --depth=1 https://github.com/chips-project/aarch64-elf gcc64
		git clone --depth=1 https://github.com/chips-project/arm-eabi gcc32
	fi
	git clone --depth=1 https://github.com/prooholic/AnyKernel3 -b master anykernel
}

build(){

	export type="${1} ${2} ${3}"

	make_clean

	if [ $2 == "-oldcam" ]
	then
		make_build "whyred_defconfig"
	else
		make_build "whyred-newcam_defconfig"
	fi
		mkzip "$KERNELNAME-$LOCALVERSION-${1}${2}${3}-$DATE.zip"
}
