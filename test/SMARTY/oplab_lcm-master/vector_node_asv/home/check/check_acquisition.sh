#!/bin/bash
# Script to check ongoing image acquisition.
# Every time a new folder YYYYMMDD_hhmmss_MISSION is created

# TODO: for each shell/python file that prints into the screen, there should be an prefix for each message,
# so it can be easily identified during debugging / operation.
# OP: define console output prefix that should appear in all console messages
# Usage: echo $output_prefix "Put your message here"
output_prefix="[check_acquisition] "

# TODO: fix variable convention consistency within all the project

## Parameters ##
#REGION_NAME=sesoko
# Retrieve mission region from configuration file
REGION_NAME=$(cat ~/mission_config/region.txt)
BASE_PATH=/media/data
DIR=`ls -td $BASE_PATH/r* | head -1`
echo $output_prefix $DIR  

SUBDIR=`ls -td $DIR/i*`

LCOUNT=`find $SUBDIR -type f -name '*LC16*' | wc -l` 
RCOUNT=`find $SUBDIR -type f -name '*RC16*' | wc -l` 
echo $output_prefix LCOUNT $LCOUNT and RCOUNT $RCOUNT

LPREV=$(cat "$DIR"/lcount.txt)
RPREV=$(cat "$DIR"/rcount.txt)

echo $output_prefix LPREV $LPREV and RPREV $RPREV

# check differences between previous counters (for both cameras L&R)
if (( $LCOUNT <= $LPREV )) || (( $RCOUNT <= $RPREV )); then
	echo $output_prefix "Image count NOT increasing. Restarting acquisition..."
	echo $output_prefix "Calling <stop_mission.sh>" 
	~/stop_mission.sh
    sleep 1
	echo $output_prefix "Calling <stop_lcm.sh>" 
	~/stop_lcm.sh
	sleep 1
	echo $output_prefix "Calling <start_lcm.sh>" 
	~/start_lcm.sh
	sleep 10 
        CURRENT_DATE=$(date '+%Y%m%d_%H%M%S')
    # add another entry in the watchman_restart log
	echo $CURRENT_DATE >> ~/watchman_restarts.txt
	~/start_mission.sh	
else
	echo $output_prefix "Image count IS increasing. Carry on"
	echo $output_prefix $LCOUNT > $DIR/lcount.txt
	echo $output_prefix $RCOUNT > $DIR/rcount.txt
fi
