#!/bin/bash

# Exit immediately if a command exits with a non-zero status.
set -e

# Enable the globstar shell option
shopt -s globstar

# Make sure we are inside the github workspace
cd $GITHUB_WORKSPACE

# Create directories
mkdir $HOME/Arduino
mkdir $HOME/Arduino/libraries

# Install Arduino IDE
export PATH=$PATH:$GITHUB_WORKSPACE/bin
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
arduino-cli config init

# Enable "unsafe" library install methods
sed -i "s/enable_unsafe_install: false/enable_unsafe_install: true/" /home/runner/.arduino15/arduino-cli.yaml

arduino-cli core update-index

# Install Arduino AVR core
arduino-cli core install arduino:avr

# Install libraries
arduino-cli lib install --git-url https://github.com/Longan-Labs/Longan_CANFD.git
arduino-cli lib install Adafruit_VL53L0X

# Compile all *.ino files for the Arduino Uno
for f in **/*.ino ; do
    arduino-cli compile -e -b arduino:avr:leonardo $f
    arduino-cli compile -b arduino:avr:leonardo --build-property "compiler.cpp.extra_flags=\"-DDEBUG\"" $f
done
