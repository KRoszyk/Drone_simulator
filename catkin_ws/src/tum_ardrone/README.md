# Package tum_ardrone_with_SGPS

This is a slight modification of the [tum_ardrone] (https://github.com/tum-vision/tum_ardrone) package. It basically operates in the same manner. It allows the use of the tum vision, as well as simplified control in a gazebo simulator via the simulated GPS.

## Installation

### with catkin

``` bash
cd catkin_ws/src
git clone https://github.com/eladden/tum_ardrone_with_SGPS.git tum_ardrone
cd ..
rosdep install tum_ardrone
catkin_make
```

## Quick start

#### Launch the nodes

``` bash
roslaunch tum_ardrone tum_ardrone_2.launch
```
## How to use

Use this package as you would use the [tum_ardrone](http://wiki.ros.org/tum_ardrone). This package has an additional parameter `useSGPS`. When this is set to ``true`` the position of the drone will be determined using the gazebo simulated GPS of the [tum_simulator](https://github.com/dougvk/tum_simulator) package. Make sure you switch `usePTAM` to ``false`` or else the vision might mess up with the position estimation. 
