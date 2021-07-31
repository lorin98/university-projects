# SO_project_with_AprilTag
Simple SLAM application embedded in the srrg2_orazio environment that detects AprilTags and builds a g2o graph.

### How to use the system

##### Prerequisites
1. Install basic Ubuntu packages to deal with Arduino and g2o
```bash
sudo apt-get install build-essential libeigen3-dev libsuitesparse-dev libqt4-dev libqglviewer-dev-qt4 arduino arduino-mk libwebsockets-dev
``` 
2. Install OpenCV library. Follow the instructions available at https://gist.github.com/arthurbeggs/06df46af94af7f261513934e56103b30
3. Download the srrg2-orazio repository from https://gitlab.com/srrg-software/srrg2_orazio
4. Download AprilTag library from https://github.com/AprilRobotics/apriltag and follow the instructions in the 'Install' section of the README.md in that repository
5. Download g2o stub following the instructions at https://github.com/OpenSLAM-org/openslam_g2o
6. An ATMega-2560 board 

##### Observations
 - This project focuses ONLY on the 36h11 tags recognition, all AprilTag families are available at https://github.com/AprilRobotics/apriltag-imgs
 - This project has been tested ONLY on Ubuntu 16.04

##### Build and flash Orazio firmware
You can build and flash the firmware on your ATMega-2560 board as follow
```bash
cd <your_path>/srrg2_orazio/srrg2_orazio/firmware_build/atmega2560
make
make orazio.hex
```

##### Camera Calibration

This point is crucial. In order to have a good pose estimation of the tags, you need to calibrate your camera device. Two kinds of approach are provided here:
1. AprilCal -> download April Robotics Toolkit at https://april.eecs.umich.edu/software/java.html and follow the instructions in the AprilCal wiki (https://april.eecs.umich.edu/wiki/Camera_suite)
2. Calibration using OpenCV -> follow the instructions at https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html

After the calibration, you'll be able to find out your camera intrinsic parameters. Feel free to replace the pre-existent values with yours in "pose_estimate.h" in the 'code' folder of this repository.

##### Compile the project
 - Add the 'code' folder of this repository in srrg2_orazio/srrg2_orazio/src/host_test
 - Replace "Makefile" in srrg2_orazio/srrg2_orazio/host_build with the one provided here
 - Compile with
```bash
cd <your_path>/srrg2_orazio/srrg2_orazio/host_build
make
```
##### Run Orazio client
Run the "orazio_client_with_apriltag" executable in srrg2_orazio/srrg2_orazio/host_build with
```bash
./orazio_client_with_apriltag [-serial-device <device>, (default: /dev/ttyACM0)]  [-joy-device <device>, (default: /dev/input/js2)]  [-camera-device <device>, (default: /dev/video0)]
```
It will record a dataset with odometries and detected tags in time and store them in a generated "log.txt" file.

##### Build the g2o graph
Run the "log_parser" executable in srrg2_orazio/srrg2_orazio/host_build. You can change the default lower bounds between two pose estimations passing them from command line as follow
```bash
./log_parser <log_file> (e.g. log.txt) [-dx <meters>, (default: 0.2)]  [-dy <meters>, (default: 0.2)]  [-dtheta <radians>, (default: 0.7854)]
```
It will generate an "out.g2o" file. If you have already installed g2o, you can build the corrispondent graph with
```bash
cd <your_path>/openslam_g2o/bin
./g2o_viewer <your_path>/out.g2o
```
Press the 'Optimize' button to see the final graph of your mapped environment.


### Utilities

In 'utils' folder you can find everything you need to calibrate your camera device and to print the 36h11 AprilTags required for this project. In particular,
 - 'apriltag' section provides:
   1. two tag mosaics to be used during AprilCal calibration process
   2. a ready-to-print postscript file that can easily be converted to pdf format with
```bash
sudo apt-get install ghostscript
ps2pdf all36h11tags.ps all36h11tags.pdf
```
 - 'opencv' section provides the opencv calibration pattern to be used during OpenCV calibration process


