### Programming a Real Self-Driving Car - System Integration Project ###

Submitted by - Vishal Rangras

##### Programming a real car to drive-by-wire, following published waypoints for the given track, while stopping at Traffic Signal lights based on their states. #####

The goals for this project are to:

1. Implement Waypoint Updater Node of ROS project which publishes the next 200 waypoints and their target velocity based on the current position, base waypoint data and state of the traffic light ahead of current position. This node works basically as a **Planning** Part of the autonomous vehicle system which plans the trajectory needed to be followed by car along with the desired velocity to be followed on that trajectory.
2. Implement DBW Node, which stands for Drive-By-Wire Node of ROS project. This node leverages a PID controller to decide value of throttle and brake, a Yaw Controller to decide value of steering of the car and a low pass filter for smoothing values of steering. It leverages the data coming from Waypoint Updater node to generate actuation signals and works as a **Control** Part of the autonomous vehicle system.
3. Implement TLDetector Node, which detects a Traffic Light and Classify its states based on the camera feed which car receives from a dashcam. This node provides its detection indication to Waypoint Updater which then update target velocity of the upcoming waypoints which are to be followed by the car and also decides if braking is needed in case light state is RED. This node serves as **Perception** Part of the autonomous vehicle system.

**Notes to Reviewer**: 

- This is an individual submission and I have not worked in a team for this project and I don't intend to run this project on Carla hardware.
- I have implemented TLDetector based on Ground Truth information which we get from state of the light published by the simulator and positions of all the traffic lights on our track which we get from YAML configuration file. I have not actually implemented an Traffic Light Detection and Classification model which can give prediction based on camera feed due to lack of time.
- Since image based TL Detection is not implemented, I understand that this project cannot be evaluated for Test Site or real SDC i.e. Carla and I prefer to get my project evaluated on the simulator itself.
- I would like to submit this project in its current state and proceed further with the graduation process.
- When time permits in future, I will revisit the project and implement the image data based Traffic light detection and recognition.

#### Attributions ####

I would like to thank Udacity SDCND Team, David Silver, Atb, Micheal Virgo, my mentor Martijn de Boer, Jeremy Shannon, Oleg Potkin, An Nguyen, Bert Ciccone, Andrew Wilkie, Kaspar Sakmann, Team Robo4, Rana Khalil, Mohan Karthik, Subodh Malgonde, Davinder Chandhok and any other SDCND fellow batch mate who I might have missed to mention.

From time to time, the above mentioned people have helped me directly or indirectly sometimes through their blogs or code snippets or references or via slack chat to proceed further with not only Capstone project of SDCND but almost every module and every project of this entire program. I am grateful to be the part of this wonderful program and receive great technical as well as non-technical guidance from my fellow batch mates, forum mentors, past project reviewers, classroom mentor, Udacity Content Team, Udacity Mentorship Team and Udacity Support Team.

#### Results ####

[image1]: ./imgs/R1.png "Result 1"

[image2]: ./imgs/R2.png "Result 2"

<h5 align="center"> Car at a RED traffic signal, stopped, waiting for singal to go green. <h5>

![alt text][image1]


<h5 align="center"> Car starts moving soon as the signal becomes GREEN. <h5>

![alt text][image2]

#### Original Instructions provided by Udacity ####

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
