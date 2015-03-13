#!/bin/bash

sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
sudo apt-get update
sudo apt-get install -y gcc-arm-none-eabi

# Install stlink
sudo apt-get install -y autoconf pkg-config libusb-1.0 git
cd ~
git clone https://github.com/texane/stlink.git
cd ~/stlink
./autogen.sh
./configure
make

# Install Oracle Java. First, remove OpenJDK if it's installed
sudo apt-get purge openjdk*
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install -y oracle-java7-installer

# Install Ecplise amd CDT
sudo apt-get install -y eclipse-platform
sudo apt-get install -y eclipse-cdt

exit
