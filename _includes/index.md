### Introduction

This page provides the supplementary material for the following paper, submitted to [2016 IEEE/RSJ International Conference on Intelligent Robots and Systems](http://www.iros2016.org/) (IROS 2016).

- Title: "UAV, Come To Me: End-to-End, Multi-Scale Situated HRI with an Uninstrumented Human and a Distant UAV"
- Authors: [Mani Monajjemi](https://mani.im), [Sepehr Mohaimenianpour](http://sepehr.im) and [Richard Vaughan](https://www.cs.sfu.ca/~vaughan/)
- Affiliation: [AutonomyLab](http://autonomylab.org/), [Simon Fraser University](http://sfu.ca)

**Important Note**: A few of the source code repositories are not public yet, therefor we provided a link to download the latest snapshots of them. As we update this page during the upcoming months, we will make those repositories public and provide more technical details and documentation.


#### Changelog

- Mar 14, 2016: Initial webpage and source code release

#### Video

TBA

#### Paper

TBA

#### Souce Code

##### "obzerver": Periodic Salient Object Detector

- [Git repository](https://github.com/AutonomyLab/obzerver)
- Branch/version: `opencv-3.0`
- Note: Requires OpenCV 3.x

##### "obzerver_ros": The ROS binding for "obzerver"

- [Download URL]()
- SHA1: 
- Note: Requires a version of ROS [visio_opencv](https://github.com/ros-perception/vision_opencv) package that is compiled against OpenCV 3.x (e.g [our fork](https://github.com/AutonomyLab/vision_opencv))

##### "bebop_autonomy": ROS driver for Parrot Bebop Drone 

- [Git repository](https://github.com/AutonomyLab/bebop_autonomy)
- Branch/version: `0.4.1`

##### "bebop_vel_ctrl": Velocity controller for Parrot Bebop Drone (ROS package)

- [Download URL]()
- SHA1:

##### "bebop_vservo": Visual servo controller for Parrot Bebop Drone (ROS package)

- [Download URL]()
- SHA1:

##### "autonomy_leds": Firmware and animation engine for DotStar LED strips (ROS package)

- [Git repository](https://github.com/AutonomyLab/autonomy_leds)
- Branch/version (firmware): `cjmcu_beetle`
- Branch/version (engine): `dev`

##### "cftld_ros": ROS wrapper for CFTld, a long-term visual tracker

- [Download URL]()
- SHA1:
- Based on [CFTld tracker](https://github.com/klahaag/CFtld)

##### "autonomy_human": Short-range Face Engagement and Optical Flow Based Gesture Detection

- [Git repository](https://github.com/AutonomyLab/autonomy_hri/tree/dev/autonomy_human)
- Branch/version: `dev`
- Note: Requires a version of ROS [visio_opencv](https://github.com/ros-perception/vision_opencv) package that is compiled against OpenCV 3.x (e.g [our fork](https://github.com/AutonomyLab/vision_opencv))

##### "bebop_hri": The behavior generator code for the paper

- [Download URL]()
- SHA1: 
