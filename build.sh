#!/bin/bash

export CONFIG_LOCALVERSION=-nAa
export KBUILD_BUILD_USER=Avina
export KBUILD_BUILD_HOST=Unix
#export TOOLCHAIN=clang
export DEVICES=ginkgo
source helper
gen_toolchain

send_msg "Start building ${LOCALVERSION} for ${DEVICES}..."

START=$(date +"%s")

END=$(date +"%s")

DIFF=$(( END - START ))

send_msg "Build succesfully in $((DIFF / 60))m $((DIFF % 60))s"
