#!/bin/bash
# Original Live Script by cybojenix <anthonydking@gmail.com>
# Menu by Caio99BR <caiooliveirafarias0@gmail.com>
# Colors by aidasaidas75 <aidaslukosius75@yahoo.com>
# Toolchains by skyinfo <sh.skyinfo@gmail.com>
# And the internet for filling in else where

# Note!
# You need to download https://github.com/TeamVee/android_prebuilt_toolchains
# Clone in the same folder as the kernel to choose a toolchain and not specify a location

# Device Choice
device_choice() {
clear
# Devices available
#
# Xperia E1
device_variants_1="D2004 D2005 D2104 D2015 D2114" device_defconfig_1="cyanogenmod_falconss_defconfig" device_name_1="Sony-XperiaE1"
# Menu
echo "${x} | ${color_green}Device choice${color_stock}"
echo
echo "1 | ${device_variants_1} | ${device_name_1}"
defconfig_updater ${device_name_1}
echo
echo "* | Any other key to Exit"
echo
read -p "  | Choice | " -n 1 -s x
case "${x}" in
	1) device_defconfig=${device_defconfig_1} device_name=${device_name_1};;
	a)
		if [ -f .config ]
		then
			echo "${x} | Working on ${device_name} defconfig!"
			make -j${build_cpu_usage} ARCH="${ARCH}" CROSS_COMPILE="${kernel_build_ccache}${CROSS_COMPILE}" savedefconfig
			mv defconfig arch/${ARCH}/configs/${device_defconfig}
		fi;;
	b)
		if [ -f .config ]
		then
			cp .config arch/${ARCH}/configs/${device_defconfig}
		fi;;
esac
if ! [ "${device_defconfig}" == "" ]
then
	echo "${x} | Working on ${device_name} defconfig!"
	make -j${build_cpu_usage} ARCH="${ARCH}" CROSS_COMPILE="${kernel_build_ccache}${CROSS_COMPILE}" ${device_defconfig}
	sleep 2
fi
}

# Toolchain Choice
toolchain_choice() {
echo "${x} | Toolchain choice"
echo
if [ -f ../android_prebuilt_toolchains/aptess.sh ]
then
	. ../android_prebuilt_toolchains/aptess.sh
else
	if [ -d ../android_prebuilt_toolchains ]
	then
		echo "  | You not have APTESS Script in Android Prebuilt Toolchain folder"
		echo "  | Check the folder"
		echo "  | We will use Manual Method now"
	else
		echo "  | You don't have TeamVee Prebuilt Toolchains"
	fi
	echo
	echo "  | Please specify a location"
	echo "  | and the prefix of the chosen toolchain at the end"
	echo "  | GCC 4.6 ex. ../arm-eabi-4.6/bin/arm-eabi-"
	echo
	echo "  | Stay blank if you want to exit"
	echo
	read -p "  | Place | " CROSS_COMPILE
	if ! [ "${CROSS_COMPILE}" == "" ]
	then
		ToolchainCompile="${CROSS_COMPILE}"
	fi
fi
}

# Kernel Build Process
kernel_build() {
if [ "${CROSS_COMPILE}" == "" ]
then
	wrong_choice
	unset CROSS_COMPILE ToolchainCompile
elif [ "${ToolchainCompile}" == "" ]
then
	wrong_choice
	unset CROSS_COMPILE ToolchainCompile
elif [ "${device_defconfig}" == "" ]
then
	wrong_choice
	unset device_name device_defconfig
elif [ "${device_name}" == "" ]
then
	wrong_choice
	unset device_name device_defconfig
elif [ ! -f .config ]
then
	wrong_choice
	unset device_name device_defconfig
else
	echo "${x} | Building ${builder} ${custom_kernel} ${custom_kernel_branch}"

	if [ $(which ccache) ]
	then
		kernel_build_ccache="ccache "
		echo "  | ${color_blue}Using CCache to build${color_stock}"
	else
		echo "  | ${color_blue}CCache not enabled${color_stock}"
	fi

	echo "  | ${color_blue}Using ${build_cpu_usage} jobs at once${color_stock}"

	start_build_time=$(date +"%s")
	make -j${build_cpu_usage} ARCH="${ARCH}" CROSS_COMPILE="${kernel_build_ccache}${CROSS_COMPILE}"
	if ! [ "$?" == "0" ]
	then
		echo "  | ${color_red}Build Failed! Exiting...${color_stock}"
		break
	fi
	sleep 2
	build_time=$(($(date +"%s") - ${start_build_time}))
	build_time_minutes=$((${build_time} / 60))
fi
}

# Updater of defconfigs
defconfig_updater() {
if [ -f .config ]
then
	if [[ "${device_defconfig}" == "${1}" || "${device_name}" == "${1}" ]]
	then
		if [ $(cat arch/${ARCH}/configs/${device_defconfig} | grep "Automatically" | wc -l) == "0" ]
		then
			defconfig_format="a | Default Linux Kernel format  | Small"
		else
			defconfig_format="b | Usual copy of .config format | Complete"
		fi
		echo "  | Update defconfig from:"
		echo "${defconfig_format}"
		echo
		echo "  | to:"
		echo "a | Default Linux Kernel format  | Small"
		echo "b | Usual copy of .config format | Complete"
		echo
	fi
fi
}

# Wrong choice
wrong_choice() {
echo "${x} | This option is not available! | Something is wrong! | Check ${color_green}Choice Menu${color_stock}!"; sleep 2
}

if [ ! "${BASH_VERSION}" ]
then
	echo "  | Please do not use ${0} to run this script, just use '. build.sh'"
elif [ -e build.sh ]
then
	# Stock Color
	color_stock=$(tput sgr0)
	# Bold Colors
	color_red=$(tput bold)$(tput setaf 1)
	color_green=$(tput bold)$(tput setaf 2)
	color_yellow=$(tput bold)$(tput setaf 3)
	color_blue=$(tput bold)$(tput setaf 4)
	# Main Variables
	custom_kernel=SSProj-CAFKernel
	builder=TeamVee-SS
	custom_kernel_branch=KK
	ARCH=arm

	while true
	do
		# Build Time
		if ! [ "${build_time}" == "" ]
		then
			if [ "${build_time_minutes}" == "" ]
			then
				menu_build_time="(${color_green}$((${build_time} % 60))s${color_stock})"
			else
				menu_build_time="(${color_green}${build_time_minutes}m$((${build_time} % 60))s${color_stock})"
			fi
		fi
		build_cpu_usage=$(($(grep -c ^processor /proc/cpuinfo) + 1))
		# Variable's
		k_version=$(cat Makefile | grep VERSION | cut -c 11- | head -1)
		k_patch_level=$(cat Makefile | grep PATCHLEVEL | cut -c 14- | head -1)
		k_sub_level=$(cat Makefile | grep SUBLEVEL | cut -c 12- | head -1)
		kernel_base="${k_version}.${k_patch_level}.${k_sub_level}"
		release=$(date +%d""%m""%Y)
		if ! [ -f .version ]
		then
			echo "0" > .version
		fi
		build=$(cat .version)
		# Menu
		clear
		echo "  | Simple Linux Kernel ${kernel_base} Build Script ($(date +%d"/"%m"/"%Y))"
		echo "  | ${builder} ${custom_kernel} ${custom_kernel_branch} Release $(date +%d"/"%m"/"%Y) Build #${build}"
		echo
		echo "  | ${color_red}Clean Menu${color_stock}"
		echo "1 | Clean Kernel"
		echo "  | ${color_green}Choice Menu${color_stock}"
		echo "2 | Set Device Defconfig ${color_green}${device_name}${color_stock}"
		echo "3 | Set Toolchain        ${color_green}${ToolchainCompile}${color_stock}"
		echo "  | ${color_yellow}Build Menu${color_stock}"
		echo "4 | Build Kernel         ${menu_build_time}"
		echo "e | Exit"
		echo
		read -n 1 -p "  | Choice | " -s x
		case ${x} in
			1)
				echo "${x} | Cleaning Kernel"
				make -j${build_cpu_usage} ARCH="${ARCH}" CROSS_COMPILE="${kernel_build_ccache}${CROSS_COMPILE}" clean mrproper
				unset device_name device_defconfig build_time;;
			2) device_choice;;
			3) toolchain_choice;;
			4) kernel_build;;
			q|e) echo "${x} | Ok, Bye!"; break;;
			*) wrong_choice;;
		esac
	done
else
	echo
	echo "  | Ensure you run this file from the SAME folder as where it was,"
	echo "  | otherwise the script will have problems running the commands."
	echo "  | After you 'cd' to the correct folder, start the build script"
	echo "  | with the . build.sh command, NOT with any other command!"
	echo
fi
