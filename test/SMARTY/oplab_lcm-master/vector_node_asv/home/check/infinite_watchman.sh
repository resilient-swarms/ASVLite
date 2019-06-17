#!/bin/bash

# endless loop that will call check_acquisition each 30 seconds
while true; do
./check_acquisition.sh &
sleep 30
done
