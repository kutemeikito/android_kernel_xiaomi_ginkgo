#!/bin/bash
CONFIG=vendor/ginkgo-perf_defconfig
DATE=$(date +"%Y%m%d-%H%M")
chat_id=-510134769543
bot_token=170274243721:AAHmjDRs-y59hF2OyRAHR8EyofLVF5dsXgQA/getUpdates
make_build(){
	print "Make kernel dtb..." green
	if [ $TOOLCHAIN == clang ]; then
		export KBUILD_COMPILER_STRING=$(clang/bin/clang --version | head -n 1 | perl -pe 's/\(http.*?\)//gs' | sed -e 's/  */ /g' -e 's/[[:space:]]*$//')
		PATH="$(pwd)/clang/bin:${PATH}"
		make O=out ARCH=arm64 $CONFIG
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
		make O=out ARCH=arm64 $CONFIG
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
		-d text="${CONFIG}"
}

send_file(){
	print "Sending file..." green
	curl -F chat_id="${chat_id}" \
		-F caption="Build succesfully! | DEVICE: ${type} | SHA1 : $(sha1sum ${file} | awk '{ print $1 }')" \
		-F document=@"${CONFIG}" \
		 https://api.telegram.org/bot"${bot_token}"/sendDocument
}

mkzip(){
	print "Generate zip file..." green

	if [ -f $(pwd)/out/arch/arm64/boot/Image.gz-dtb ]; 
	[ -f $(pwd)/out/arch/arm64/boot/dtbo.img ];
	then
		cp $(pwd)/out/arch/arm64/boot/Image.gz-dtb anykernel
		cp $(pwd)/out/arch/arm64/boot/dtbo.img anykernel
		cd anykernel
		zip -r9 ../$1 * -x .git README.md *placeholder
		cd ..
		send_file $CONFIG
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
		echo -e "\033[31m $CONFIG \033[0m";;
		"green")
		echo -e "\033[32m $CONFIG \033[0m";;
		*)
		echo $CONFIG
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
	git clone --depth=1 https://github.com/avinakefin/Anykernel anykernel
}

build(){

	make_clean
	
		make_build "$CONFIG"
		mkzip "$LOCALVERSION-$DATE.zip"
}