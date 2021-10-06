#!/bin/bash -eu

array=(
	rtled0
	rtled1
	rtled2
	rtled3
	rtswitch0
	rtswitch1
	rtswitch2
	rtbuzzer0
	rtlightsensor0
	rtmotoren0
	rtmotor_raw_r0
	rtmotor_raw_l0
	rtmotor0
	rtcounter0
	rtcounter_r0
	rtcounter_l0
)

for targetfile in ${array[@]} ; do
	sudo touch /dev/$targetfile
	sudo chmod 666 /dev/$targetfile
	echo 0 > /dev/$targetfile
done

echo 1 > /dev/rtswitch0
echo 1 > /dev/rtswitch1
echo 1 > /dev/rtswitch2
