# PiCar

PiCar is a car platform for testing autonomous driving software.

## Photo 
- will be added

## List of Used Components
- [Raspberry Pi Model 3 B+](https://www.raspberrypi.org/products/raspberry-pi-3-model-b-plus/)
- [Raspberry Pi Camera v2](https://www.raspberrypi.org/products/camera-module-v2/)
- [HC-SR04 Ultrasonic Distance Sensor](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
- 1k and 2k Resistors
- [RPi Motor Driver Board](https://www.waveshare.com/wiki/RPi_Motor_Driver_Board)
- 11.1 V 3S 2250mAh 35C Li-po Battery
- Mini Pan Tilt Kit
- 2 x [SG90 9G Mini Servo Motor](http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf)
- 4WD Robot Car Platform
- A3 Compact Li-po (2-3S) Charger for Battery
- Jumper Wires

## Required Software Libraries
- [Raspicam](https://www.uco.es/investiga/grupos/ava/node/40)
- [OpenCV](https://opencv.org/)
- [WiringPi](http://wiringpi.com/)

## Circuit Diagram 
- Will be added


## Usage
1. Install git on Raspberry Pi. `sudo apt install -y git`
2. Clone this repo. `git clone https://github.com/oguztoraman/picar`
3. Run setup.sh. `cd picar && bash setup.sh`
4. Install the latest version of Raspicam from [here](https://sourceforge.net/projects/raspicam/files/) on Raspbery Pi.
5. Write your autonomous driving test code in main.cpp and compile whole project with `make all`
6. Run your code with `./picar`

## License
- GPLv3. See the COPYING file for details.

## References
- [Raspberry Pi Pinout](https://pinout.xyz/pinout/wiringpi#)
- [Usage of HC-SR04 Ultrasonic Distance Sensor on Raspberry Pi](https://thepihut.com/blogs/raspberry-pi-tutorials/hc-sr04-ultrasonic-range-sensor-on-the-raspberry-pi)
- [Usage of Raspicam](https://www.uco.es/investiga/grupos/ava/node/40)
- [Cppreference](https://en.cppreference.com/w/)
