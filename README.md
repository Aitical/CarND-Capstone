# CarND-Capstone
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013)
### System Architecture  
This is the Capstone project for the Udacity Self-Driving Car Nanodegree. We developed software to guide a real self-driving car around a test track. Using the Robot Operating System (ROS), we created nodes for traffic light detection and classification, trajectory planning, and control.
![pic referenced from Udacity](imgs/arc.png)
### Team Members: 

* **** [github](), [emai]() - team leader
* **** [github](), [email]()
* **Wu Gang** [github](https://github.com/fadida), [email](w965813422@gmail.com)

### Project Components

#### Traffic Light Detector and Classifier
Car receives image from the camera, system can detect and classify a traffic light color, if the traffic light is not detected the None is returned. We built two models. First part is to detect a traffic light  using the UNet and the second part is to classify using our own network.

##### Traffic light Detection
We use the segmentation method to get detect the traffic light. We used [UNet](https://arxiv.org/pdf/1505.04597.pdf) to segment and used the keras to build model. During building the model, we learned a lot from this [repo](https://github.com/zhixuhao/unet) which provided a useful model code and we used a previously trained model and weights then fine tuning on our own labled data and [Bosch](https://hci.iwr.uni-heidelberg.de/node/6132) dataset.

##### Traffic light Classifier

To classify the type of the traffic light, we used the knowledge learned in the last term and built a simple but efficiency CNN model with FC Layer to classify the traffic lights. Details of the model showing in the following table:
![pic referenced from Udacity](https://raw.githubusercontent.com/Aitical/CarND-Capstone/master/imgs/cls.png)

**For more detail about the detector can be found in [detector](https://github.com/Aitical/CarND-Capstone/tree/master/detector)**

##### Waypoint updater
 - Waypoint updater performs the following at each current pose update
 - Find closest waypoint
   - This is done by first searching for the waypoint with closest 2D Euclidean distance to the current pose among the waypoint list
   - Once the closest waypoint is found it is transformed to vehicle coordinate system in order to determine whether it is ahead of the vehicle and advance one waypoint if found to be behind
   - Searching for the closest waypoint is done by constructing a k-d tree of waypoints at the start with x and y coordinates as dimensions used to partition the point space. This makes the search O(log n) in time complexity. In practice we reduced the search time on average from 8.6ms to 0.14ms.
 - Calculate trajectory
   - The target speed at the next waypoint is calculated as the expected speed (v) at the next waypoint so that the vehicle reaches 0 speed after traversing the distance (s) from the next waypoint to the traffic light stop line and the largest deceleration (a)
   - Using linear motion equations it can be shown that v = sqrt(2 x a x s)
   - If there is no traffic light stopline, then target speed is set to the maximum
 - Construct final waypoints
   - Published final waypoints are constructed by extracting the number of look ahead waypoints starting at the calculated next waypoint
   - The speeds at published waypoints are set to the lower of target speed and maximum speed of the particular waypoint

##### DBW
 - Throttle and brake is controlled via PID controller
 - PID controller inputs
   - reference - linear x velocity of the twist command from waypoint follower
   - measurement - linear x velocity of the current velocity
 - PID control output, when positive, is converted to throttle by clipping it between 0.0 and 1.0
 - PID control output, when negative, is converted to brake by first normalizing it and then using it to modulate the max braking torque
   - The normalizer is approximated by (|Kp|+|Kd|+|Ki|) x |max input|, where Kx are PID gains and max input is the speed limit
   - Maximum braking torque is calculated as (total vehicle mass) x (deceleration limit) x (wheel radius)

 - Required steering angle is calculated using the linear x velocity and angular z velocity of the twist command from waypoint follower and current linear x velocity using the provided yaw controller
   - steering angle = arctan(wheel base / turning radius) x steer ratio
   - turning radius = current linear velocity / target angular velocity
   - target angular velocity  = current angular velocity x (current linear velocity / target linear velocity)
 - Steering command is filtered with a low pass filter to avoid fast steering changes


###### This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

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