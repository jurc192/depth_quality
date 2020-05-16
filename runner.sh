#!/bin/bash

# EXPERIMENT 10
# Exposure vs laserpower
# Each measurement 10 times
# One distance = 10 .csv files

nframes=50

# DISTANCE 20 CM ##############################################################
distance=20
dstfolder=$1

# Just a preview
python depth_acquisition.py 20 8500 150 $dstfolder 1 0

for exp in 8500 7500 6500 5500 4500 3500; do
    for lpow in 150 180 210 240; do

        printf "Exposure $exp, laserpower $lpow\n"
        for i in `seq $nframes`; do
            printf "Capturing frame $i at $distance cm\n"
            python depth_acquisition.py $distance $exp $lpow $dstfolder 0 $i
            if [ $? -ne 0 ]; then
                printf "Reconnect the camera " && read dummy && python depth_acquisition.py $distance $exp $lpow $dstfolder 0 $i
            fi
        done
        printf "\n"

    done
done


## analysis
python depth_analysis.py "$dstfolder/$distance" exp_lpow.csv $nframes $distance
