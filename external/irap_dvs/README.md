# irap_dvs

Dynamic vision sensor tools for IRAP


- prerequisites


1. Dependencies :

`$ sudo apt-get install libusb-1.0-0-dev`

`$ sudo apt-get install ros-kinetic-camera-info-manager ros-kinetic-image-view python-catkin-tools`


2. Download catkin_simple :

`$ git clone https://github.com/catkin/catkin_simple`


3. For DAIVS240C, Download ros dvs driver from rpg :

`$ git clone https://github.com/uzh-rpg/rpg_dvs_ros`

The ROS DVS package provides C++ drivers for the [Dynamic Vision Sensor (DVS)](https://inilabs.com/products/dynamic-vision-sensors/) and the [Dynamic and Active-pixel Vision Sensor (DAVIS)](https://inilabs.com/products/dynamic-and-active-pixel-vision-sensor/).


4. Prior to executing catkin_make, compile libcaer_catkin

`$ cd rpg_dvs_ros/libcaer_catkin/`

`$ mkdir build && cd build && cmake ..`

`$ sudo make && make install`

`$ sudo ./install.sh`

5. For samsung dvs, download cyusb and install

source file : http://www.cypress.com/file/139281 (need id and password)

**After install, You need to manually copy include(*.h) files from cyusb include folder to /usr/local/include/**

6. Build catkin workspace

`$ cd ~/catkin_ws && catkin_make`

7. If an error occurs, download libcaer package and compile again.

Please refer to [libcaer](https://github.com/inilabs/libcaer) page.

**Even if you see fail messages, do not cancel and let catkin_make finish the job. it automatically links the missing links during compialition.**

8. launch davis and check rostopic list.

`$ roslaunch dvs_renderer davis_mono.launch`

9. Then you are ready. download irap_dvs pkg and compile.

## fome (with gui)

- Fast Object & Movement detection with Event Camera

`$ roslaunch fome fome_gui.launch`

## samsungdvs

- Viewer

`$ rosrun samsungdvs dvsbulk -l ~/catkinws/src/irap_dvs/samsung/config/run_dvs_fx3_2_default.txt`

`$ rosrun samsungdvs dvsview`
