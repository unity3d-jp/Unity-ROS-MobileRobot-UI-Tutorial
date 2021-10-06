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

[[ -z $(lsmod | grep rtmouse) ]] || exit 0
for targetfile in ${array[@]} ; do
	[[ -e /dev/$targetfile ]] && sudo rm /dev/$targetfile
done
