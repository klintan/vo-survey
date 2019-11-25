# A survey of ROS enabled Visual odometry (and VSLAM)
I've been trying to find a ROS2 package for visual odometry that publishes an odometry topic, and it turned out to be quite difficult. I decided to to this little write-up for others interested in the same thing, perhaps it'll make it easier for someone.

Contributions and updates are appreciated! Preferably a PR, if not create an issue if you see any problems.

Beware and avoid the GPL licensed stuff, mostly just there for reference and for people in research.

## Definitions of terms:
- Odometry - in ROS twist (rotation) and pose (translation) of the robot
- Visual Inertial SLAM (VI-SLAM) - is SLAM based on both visual (camera) sensor information and IMU (inertial information) fused.
- Monocular visual odometry - Odometry based on a single (mono) camera.
- Wheel odometry - using the size and angular motion (rotation) of the robots wheels calculate how the robot is moving.
 

## Packages and code

### SLAM

- cartographer - Real-time simultaneous localization and mapping (SLAM) in 2D and 3D across multiple platforms and sensor configurations. cartographer



### Visual Odometry

*Active* is used loosely and subjectively, pretty much if I see that either some PR has been merged, commit has been made or even some maintainer/author has addressed an issue the past 9 - 12 month I consider it active. 




| Package  |  Active/Stars | Topics published |  Stereo | Mono  |  Omni | SLAM | ROS version | Year | Paper |  License | 
|---|---|---|---|---|---|---|---|---|---|---|
| [Learn VI-ORB](https://github.com/jingpang/LearnVIORB) | 547 / No | None |  Yes |  Yes |  No | Yes  | 1 |	2016| https://arxiv.org/abs/1610.06475 ||
| [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) | 4500 / No|None  |  Yes |  Yes | No  |  Yes | 1  | 2016|	https://arxiv.org/abs/1610.06475 ||
| [OpenVSLAM](https://github.com/xdspacelab/openvslam)  | 1700 / Yes | None |  Yes |  Yes |  No | Yes  |  1 / 2  |	2019 |	https://arxiv.org/abs/1910.01122 | |
| [VISO2](https://github.com/srv/viso2) |143 / No | nav_msgs/Odometry / geometry_msgs/Pose| Yes | Yes | Yes | No | 1 | http://t.cvlibs.net/publications/Geiger2011IV.pdf ||
| [XIVO](https://github.com/ucla-vision/xivo) | 383 / Yes | None | Yes | Yes | No | N/A | 1 | 2019 | http://vision.ucla.edu/papers/tsotsosCS15.pdf ||
| [Rovio](https://github.com/ethz-asl/rovio) | 643 / Yes | None |  Yes | Yes | No | N/A | 1 | 2017 | https://www.research-collection.ethz.ch/handle/20.500.11850/263423 | |
|[Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) | 126  / Yes | None | Yes | Yes | No | N/A | 1 | 2019 | https://arxiv.org/pdf/1910.02490.pdf ||
| [LSD-SLAM](https://github.com/tum-vision/lsd_slam) | 1900 / No | None | No | Yes | No | N/A | 1 | 2014 | https://vision.in.tum.de/_media/spezial/bib/caruso2015_omni_lsdslam.pdf |GPLv3 |
|[CubeSLAM](https://github.com/shichaoy/cube_slam) | 267 / Yes | None | Yes | Yes | No | Yes | 1 | 2019 | https://arxiv.org/abs/1806.00557 | BSD |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) | 976 / Yes | None | Yes | Yes | No | Yes | 1 | 2019 | https://ieeexplore.ieee.org/abstract/document/8593603 | GPLv3 |
| [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) | 2100 / Yes | None | Yes | Yes | No | No | 1 | 2017 | https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert | GPLv3 |
| [OpenVINS](https://github.com/rpng/open_vins) | 283 / Yes | None | Yes | Yes | No | N/A | 1 | 2019 | https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf | GPLv3 |
| [SE2SLAM](https://github.com/izhengfan/se2lam) | 152 / Yes | geometry_msgs::Pose / geometry_msgs::PoseStamped | Yes | Yes | No | Yes | 1 | 2019 | https://fzheng.me/icra/2019.pdf | MIT |
| [SE2CLAM](https://github.com/izhengfan/se2clam) | 69 / Yes | geometry_msgs::PoseStamped | Yes | Yes | No | Yes | 1 | 2018 |  https://ieeexplore.ieee.org/document/8357438 | MIT | 
| [VINS-FusionGPU](https://github.com/pjrambo/VINS-Fusion-gpu)