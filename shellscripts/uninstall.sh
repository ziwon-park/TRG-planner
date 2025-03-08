#!/bin/bash

LIB_DIR="/usr/local/lib"
INCLUDE_DIR="/usr/local/include"
CMAKE_DIR="/usr/local/lib/cmake"

LIB_NAME="trg_planner"

# Warning message
echo -e "\e[1;33mWarning: This will delete the following files from your system:\e[0m"
echo -e "\e[1;33m- All files related to ${LIB_NAME} in ${LIB_DIR}\e[0m"
echo -e "\e[1;33m- All files related to ${LIB_NAME} in ${INCLUDE_DIR}\e[0m"
echo -e "\e[1;33m- All files related to ${LIB_NAME} in ${CMAKE_DIR}\e[0m"
echo -e "\e[1;33m- Are you sure you want to continue? [y/n]\e[0m"
read -r RESPONSE

if [[ "$RESPONSE" != "y" && "$RESPONSE" != "Y" ]]; then
    echo "Uninstallation aborted."
    exit 0
fi

# Remove library files
echo "Removing library files..."
sudo rm -f ${LIB_DIR}/lib${LIB_NAME}_core.a
sudo rm -f ${LIB_DIR}/lib${LIB_NAME}_core.so
sudo rm -f ${LIB_DIR}/lib${LIB_NAME}_core.*

echo "Removing include files..."
sudo rm -rf ${INCLUDE_DIR}/${LIB_NAME}

echo "Removing CMake files..."
sudo rm -rf ${CMAKE_DIR}/${LIB_NAME}

echo "Uninstallation complete."
