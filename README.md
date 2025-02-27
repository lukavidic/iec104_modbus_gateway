# About the project
This project implements a simple gateway for IEC 104 to MODBUS RTU protocol communication. Project is created for a custom target platform based on NUC980DK61YC microprocessor manufactured by Nuvoton. Applications are written for Linux operating system that will run on the target platform.
Project is made as a part of a grade for university subject Reliability of electronic systems on the second study cycle.

## Structure of the repository
The repository contains the following directories and files:
- MA35D1_Buildroot_master - contains files needed for kernel generation using buildroot build system
- docs - place where Doxygen documentation will be stored after running doxygen command with the provided Doxyfile
- modbus_master - folder that cointains implementation of modbus master used for this project, also has a test application that tests the functionality of implemented modbus master
- project - contains iec60870 library files and gateway application source file
- Doxyfile - Doxygen configuration file needed for documentation generation
- Izvjestaj.pdf (Izvjestaj.docx) - project report written in native (Serbian) language
- old_uboot_env.txt - old U-Boot enviroment variables found on factory U-Boot settings
- protocol_mapping.txt - text file that contains mapping between the two protocols of interest
- set-environment.sh - a script file that needs to be sourced before cross-compiling applications

## How to create Linux kernel image
To create Linux kernel image that needs to run on the target platform, apply the following steps:
- Clone [buildroot fork repository](https://github.com/OpenNuvoton/MA35D1_Buildroot) created by Nuvoton company
- Copy all files from the MA35D1_Buildroot_master folder of this repo to the folder of the previously cloned repo
- Apply the custom defconfig using _make nuc980_custom_defconfig_
- Build images using _make_ command
- After build completed successfully, generated files will be located in _output/images_ folder of MA35D1 buildroot folder

## How to deploy kernel image and device tree blob (.dtb)

One option is to use Nuvoton NuWriter programming tool with the generated files (_uImage, nuc980-custom.dtb_). Second option is to boot system via network if you have factory U-Boot running and cannot access USB Boot mode for NuWriter to work.
To boot via network, first install tftp server locally with command _sudo apt install tftpd-hpa_. On the target platform, stop the booting process while in U-Boot and enter the U-Boot shell. Inside U-Boot shell, create appropriate network configuration and modify _bootcmd_ command to the following: _tftp 0x7FC0 uImage; tftp 0xA00000 nuc980.dtb; bootm 0x7FC0 - 0xA00000_.
**Note**: There is a buildroot post-build script that will automatically copy kernel image file and .dtb file to the /srv/tftp folder (requires sudo mode at the end of every build).

## How to compile and run test applications
First step to compiling any application is to source the environment script located in the root of this repo. Use command _source set-environment.sh_.
After building any application, transfer the executable file to the target platform using ssh or buildroot root filesystem overlay mechanism. Connect to the target platform and run the app like any other executable.

### Compile modbus master test app
Locate shell to the modbus_master folder of this repo. Execute a simple cross-compiling command: _arm-linux-gcc *.c -o modbus_master -ljansson -lmodbus_

### Compile gateway application
First build the iec60870 library. Locate shell into project folder and use the following command: _make CC=arm-linux-gcc AR=arm-linux-ar_. Then proceed to build the application. Locate shell into project/examples/cs104_server and use: _make CC=arm-linux-gcc_.

## Additional notes
This project is made for the custom commercial NUC980 board. If you have another board you will probably need to modify the device tree source file (_nuc980-custom.dts_) to match your hardware configuration.
To enable ssh, you need to create a pair of RSA keys locally and copy the public key content to a file named _authorized_keys_. Place this file in buildroot overlay folder inside _/root/.ssh/_.
If you want to change network configuration of the target platform in Linux you can do so by modifying _/etc/network/interfaces_ file located inside the overlay folder.
