### Introduction

This page provides supplementary materials for the following paper, presented in [2016 IEEE/RSJ International Conference on Intelligent Robots and Systems](http://www.iros2016.org/) (IROS 2016).


<p class="message">
**"UAV, Come To Me: End-to-End, Multi-Scale Situated HRI with an Uninstrumented Human and a Distant UAV"**
[Mani Monajjemi](https://mani.im), [Sepehr Mohaimenianpour](http://sepehr.im) and [Richard Vaughan](https://www.cs.sfu.ca/~vaughan/)
_[AutonomyLab](http://autonomylab.org/), [Simon Fraser University](http://sfu.ca)_
</p>

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

- [Download URL](https://github.com/AutonomyLab/bebop_hri/releases/download/iros16_submission/AutonomyLab-obzerver_ros-23e7d29fa8b3.tar.gz)
- Snapshot commit hash: `23e7d29fa8b3`
- SHA1: `a028eb811aa0bfa1f342dba07eef820c6a18db76`
- Note: Requires a version of ROS [vision_opencv](https://github.com/ros-perception/vision_opencv) package that is compiled against OpenCV 3.x (e.g [our fork](https://github.com/AutonomyLab/vision_opencv))

##### "bebop_autonomy": ROS driver for Parrot Bebop Drone

- [Git repository](https://github.com/AutonomyLab/bebop_autonomy)
- Branch/version: `0.4.1`

##### "bebop_vel_ctrl": Velocity controller for Parrot Bebop Drone (ROS package)

- [Download URL](https://github.com/AutonomyLab/bebop_hri/releases/download/iros16_submission/AutonomyLab-bebop_vel_ctrl-4962f392b33c.tar.gz)
- Snapshot commit hash: `4962f392b33c`
- SHA1: `f9f95f75af1efed9da1ca08e3ad06c24081276d9`

##### "bebop_vservo": Visual servo controller for Parrot Bebop Drone (ROS package)

- [Download URL](https://github.com/AutonomyLab/bebop_hri/releases/download/iros16_submission/AutonomyLab-bebop_vservo-f25bfaaf74d0.tar.gz)
- Snapshot commit hash: `f25bfaaf74d0`
- SHA1: `161dbda80075720f07936241b0d8780e3caab41e`

##### "autonomy_leds": Firmware and animation engine for DotStar LED strips (ROS package)

- [Git repository](https://github.com/AutonomyLab/autonomy_leds)
- Branch/version (firmware): `cjmcu_beetle`
- Branch/version (engine): `dev`

##### "cftld_ros": ROS wrapper for CFTld, a long-term visual tracker

- [Download URL](https://github.com/AutonomyLab/bebop_hri/releases/download/iros16_submission/AutonomyLab-cftld_ros-68eb0b0774ae.tar.gz)
- Snapshot commit hash: `378029e2b0eb0c61c555710838844cd13e026aab`
- SHA1: `378029e2b0eb0c61c555710838844cd13e026aab`
- Based on [CFTld tracker](https://github.com/klahaag/CFtld)

##### "autonomy_human": Short-range Face Engagement and Optical Flow Based Gesture Detection

- [Git repository](https://github.com/AutonomyLab/autonomy_hri/tree/dev/autonomy_human)
- Branch/version: `dev`
- Note: Requires a version of ROS [vision_opencv](https://github.com/ros-perception/vision_opencv) package that is compiled against OpenCV 3.x (e.g [our fork](https://github.com/AutonomyLab/vision_opencv))

##### "bebop_hri": The behavior generator code for the paper

- [Download URL](https://github.com/AutonomyLab/bebop_hri/releases/download/iros16_submission/AutonomyLab-bebop_hri-2ea50ade4e88.tar.gz)
- Snapshot commit hash: `2ea50ade4e88`
- SHA1: `45565286448428879fa46795f6cefa7163c1b8eb`
