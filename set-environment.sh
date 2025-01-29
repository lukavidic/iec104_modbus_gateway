#!/bin/bash

export PATH=${HOME}/MA35D1_Buildroot-master/output/host/bin:$PATH
export CROSS_COMPILE=arm-linux-
export ARCH=arm
export SYSROOT=$(arm-linux-gcc -print-sysroot)

