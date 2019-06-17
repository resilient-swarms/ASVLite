#!/bin/bash

# Script to start ASV node mission
# Every time a new folder YYYYMMDD_hhmmss_MISSION is created

# include parse_yaml helper function
source parser/parse_yaml.sh

# TODO: for each shell/python file that prints into the screen, there should be an prefix for each message,
# so it can be easily identified during debugging / operation.
# OP: define console output prefix that should appear in all console messages
# Usage: echo $output_prefix "Put your message here"
output_prefix="[start_mission] "

# read yaml file from configuration folder
config_dir="/home/pi/configuration"

# specify path where output data will be stored
# modify to your own local structure when running in a PC, instead in the RPi 
output_path=/home/pi/data

# specify root folder for lcm
lcm_root="/home/pi/git/oplab_lcm/vector_node_asv"
# modify to your own local structure when running in a PC, instead in the RPi 

echo
echo $output_prefix "Reading YAML configuration files"

# mission.yaml contains information about behaviour, waypoints, etc
eval $(parse_yaml ${config_dir}/mission.yaml "mission_")
# vehicle.yaml contains the description of vehicle properties, controller, sensor, etc
eval $(parse_yaml ${config_dir}/vehicle.yaml "vehicle_")

# access yaml content
mission_name="$(echo -e "${mission_mission}" | tr -d '[:space:]')"
platform_name="$(echo -e "${vehicle_platform}" | tr -d '[:space:]')"
log_format="$(echo -e "${vehicle_log_format}" | tr -d '[:space:]')"

echo -e $output_prefix "PLATFORM:  ${platform_name}"
echo -e $output_prefix "MISSION:   ${mission_name}"
echo -e $output_prefix "LOG_FORMAT:${log_format}"

date_time=$(date '+%Y%m%d_%H%M%S')

# folder for the specific platform, given the platform name
origin_dir="${lcm_root}/platforms/$platform_name"
# keeps track of numer of iterations for the current mission name
count_file="${origin_dir}/${mission_name}.count"

if [ -f $count_file ]; then
	# if mission counter exists, increase by 1
    count=$((`cat $count_file` + 1))
else
	# if not, create a new dir for this platform, and start counter from 1
    mkdir -p $origin_dir
    count=1
fi

echo
echo $count > $count_file
counter=`printf "%03d" $count`

# check if output_dir is empty
output_dir="${output_path}/${date_time}_${platform_name}_${mission_name}_${counter}"

# create the DIR and all subdirectories if necessary without prompting the user
mkdir -p $output_dir
# check is there is any file inside the output directory
fn=$(ls -l $output_dir | wc -l)
if [ $fn -gt 1 ]; then
    echo -e $output_prefix "WARNING: Directory $output_dir is not empty, aborting mission start";
    exit
fi;

#log lcm info
lcm-logger -v -c ${log_format} ${output_dir}/lcmlog_${date_time}.lcm> /dev/null 2> /dev/null &
echo -e $output_prefix "lcm-logger started in background"

#initialise camera counts
echo 0 > $output_dir/image_count.txt
echo -e $output_prefix "image counter initialised"

# include script calls here
echo
echo -e $output_prefix "MISSION"
echo
echo -e $output_prefix "<config_dir> set as: ${config_dir}"
echo -e $output_prefix "<output_dir> set as: ${output_dir}"

# TODO: check for more general path handling

python ~/git/oplab_lcm/vector_node_asv/src/asv_main.py start -c ${config_dir} -o ${output_dir} &

# start infinite watchman in background
#echo "Starting infinite watchman... wait 10 seconds"
#sleep 10; ~/check/infinite_watchman.sh > /dev/null &
#echo "Automatically restart acquisition if camera stop saving images"
