#!/bin/bash

# Download the file
DATA_DIR="../prebuilt_maps"
mkdir -p $DATA_DIR

URL=""
# URL="https://urserver.kaist.ac.kr/publicdata/gazebo-isaacsim-plugin/forest.zip"

wget -c $URL -P $DATA_DIR

# Unzip the file
unzip $DATA_DIR/prebuilt_maps.zip -d $DATA_DIR

# Remove the zip file
rm $DATA_DIR/prebuilt_maps.zip
