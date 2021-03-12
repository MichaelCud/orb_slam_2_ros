# ORB-SLAM2
**ORB-SLAM2 Authors:** [Raul Mur-Artal](http://webdiis.unizar.es/~raulmur/), [Juan D. Tardos](http://webdiis.unizar.es/~jdtardos/), [J. M. M. Montiel](http://webdiis.unizar.es/~josemari/) and [Dorian Galvez-Lopez](http://doriangalvez.com/) ([DBoW2](https://github.com/dorian3d/DBoW2)).
The original implementation can be found [here](https://github.com/raulmur/ORB_SLAM2.git).

# ORB-SLAM2 ROS node
This is the ROS implementation of the ORB-SLAM2 real-time SLAM library for **Monocular**, **Stereo** and **RGB-D** cameras that computes the camera trajectory and a sparse 3D reconstruction (in the stereo and RGB-D case with true scale). It is able to detect loops and relocalize the camera in real time. This implementation removes the Pangolin dependency, and the original viewer. All data I/O is handled via ROS topics. For vizualization you can use RViz. This repository is maintained by [Lennart Haller](http://lennarthaller.de) on behalf of [appliedAI](http://appliedai.de).
## Features
- Full ROS compatibility
- Supports a lot of cameras out of the box, such as the Intel RealSense family. See the run section for a list
- Data I/O via ROS topics
- Parameters can be set with the rqt_reconfigure gui during runtime
- Very quick startup through considerably sped up vocab file loading
- Full Map save and load functionality based on [this PR](https://github.com/raulmur/ORB_SLAM2/pull/381).
- Loading of all parameters via launch file
- Supports loading cam parameters from cam_info topic

### Related Publications:
[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.

[Stereo and RGB-D] Raúl Mur-Artal and Juan D. Tardós. **ORB-SLAM2: an Open-Source SLAM System for Monocular, Stereo and RGB-D Cameras**. *IEEE Transactions on Robotics,* vol. 33, no. 5, pp. 1255-1262, 2017. **[PDF](https://128.84.21.199/pdf/1610.06475.pdf)**.

[DBoW2 Place Recognizer] Dorian Gálvez-López and Juan D. Tardós. **Bags of Binary Words for Fast Place Recognition in Image Sequences**. *IEEE Transactions on Robotics,* vol. 28, no. 5, pp.  1188-1197, 2012. **[PDF](http://doriangalvez.com/php/dl.php?dlp=GalvezTRO12.pdf)**

# 1. License
ORB-SLAM2 is released under a [GPLv3 license](https://github.com/aaide/ORB_SLAM2_ROS/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/aaide/ORB_SLAM2_ROS/blob/master/Dependencies.md).

For a closed-source version of ORB-SLAM2 for commercial purposes, please contact the authors: orbslam (at) unizar (dot) es.

If you use ORB-SLAM2 (Monocular) in an academic work, please cite:

    @article{murTRO2015,
      title={{ORB-SLAM}: a Versatile and Accurate Monocular {SLAM} System},
      author={Mur-Artal, Ra\'ul, Montiel, J. M. M. and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={31},
      number={5},
      pages={1147--1163},
      doi = {10.1109/TRO.2015.2463671},
      year={2015}
     }

if you use ORB-SLAM2 (Stereo or RGB-D) in an academic work, please cite:

    @article{murORB2,
      title={{ORB-SLAM2}: an Open-Source {SLAM} System for Monocular, Stereo and {RGB-D} Cameras},
      author={Mur-Artal, Ra\'ul and Tard\'os, Juan D.},
      journal={IEEE Transactions on Robotics},
      volume={33},
      number={5},
      pages={1255--1262},
      doi = {10.1109/TRO.2017.2705103},
      year={2017}
     }

# 2. Building orb_slam2_ros
We have tested the library in **Ubuntu 16.04** with **ROS Kinetic** and **Ubuntu 18.04** with **ROS Melodic**. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.
A C++11 compiler is needed.

## Getting the code
Clone the repository into your catkin workspace:
```
git clone https://github.com/MichaelCud/orb_slam_2_ros.git
```

# 3. Prerequisites
see all prerequisites and requirements here: https://github.com/appliedAI-Initiative/orb_slam_2_ros and here: https://github.com/raulmur/ORB_SLAM2


Current forked repo is used for academic project "Treasure Hunter"
This repo is used AR files which had been taken from here: https://github.com/raulmur/ORB_SLAM2/tree/master/Examples/ROS/ORB_SLAM2/src
and modified for the project purpeses.
