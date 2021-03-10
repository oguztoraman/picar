#! /bin/bash
#
#  Setup.sh
#   
#  A bash script that updates the system and installs necessary software packages for PiCar
#   
echo "Updating the system..."
sudo apt update
sudo apt upgrade -y
echo "Installing necessary softwares..."
sudo apt install -y g++ libopencv-dev make wiringpi
echo "Setup finished."
