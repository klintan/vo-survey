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

| Package  |  Active | License | Topics published |  Stereo | Mono  |  Omni | SLAM | ROS version | Year | Paper  | 
|---|---|---|---|---|---|---|---|---|---|---|
| [Learn VI-ORB](https://github.com/jingpang/LearnVIORB) ![Learn VI-ORB](https://img.shields.io/github/stars/jingpang/LearnVIORB.svg)| No | GPLv3 | None |  Yes |  Yes |  No | Yes  | 1 |	2016| https://arxiv.org/abs/1610.06475 |
| [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) ![ORB-SLAM2](https://img.shields.io/github/stars/raulmur/ORB_SLAM2.svg)| No | GPLv3 | None |  Yes |  Yes | No  |  Yes | 1  | 2016|	https://arxiv.org/abs/1610.06475 |
| [OpenVSLAM](https://github.com/xdspacelab/openvslam)  ![OpenVSLAM](https://img.shields.io/github/stars/xdspacelab/openvslam.svg)| Yes | MIT | None |  Yes |  Yes |  No | Yes  |  1 / 2  |	2019 |	https://arxiv.org/abs/1910.01122 | 
| [VISO2](https://github.com/srv/viso2) ![VISO2](https://img.shields.io/github/stars/srv/viso2.svg)| No | GPLv3 | nav_msgs/Odometry / geometry_msgs/Pose| Yes | Yes | Yes | No | 1 | 2011 | http://t.cvlibs.net/publications/Geiger2011IV.pdf |
| [XIVO](https://github.com/ucla-vision/xivo) ![XIVO](https://img.shields.io/github/stars/ucla-vision/xivo.svg)| Yes | GPLv3/CC BY-NC 3.0-like | None | Yes | Yes | No | N/A | 1 | 2019 | http://vision.ucla.edu/papers/tsotsosCS15.pdf |
| [Rovio](https://github.com/ethz-asl/rovio) ![Rovio](https://img.shields.io/github/stars/ethz-asl/rovio.svg)| Yes | BSD-3 |None |  Yes | Yes | No | N/A | 1 | 2017 | https://www.research-collection.ethz.ch/handle/20.500.11850/263423 | MIT-like |
|[Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) ![Kimera-Semantics](https://img.shields.io/github/stars/MIT-SPARK/Kimera-Semantics.svg)| Yes | BSD-2 |None | Yes | Yes | No | N/A | 1 | 2019 | https://arxiv.org/pdf/1910.02490.pdf |
| [LSD-SLAM](https://github.com/tum-vision/lsd_slam) ![LSD-SLAM](https://img.shields.io/github/stars/tum-vision/lsd_slam.svg)| No | GPLv3 | None | No | Yes | No | N/A | 1 | 2014 | https://vision.in.tum.de/_media/spezial/bib/caruso2015_omni_lsdslam.pdf |
|[CubeSLAM](https://github.com/shichaoy/cube_slam) ![CubeSLAM](https://img.shields.io/github/stars/shichaoy/cube_slam.svg)| Yes |  BSD-3 | None | Yes | Yes | No | Yes | 1 | 2019 | https://arxiv.org/abs/1806.00557 |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) ![VINS-Fusion](https://img.shields.io/github/stars/HKUST-Aerial-Robotics/VINS-Fusion.svg)| Yes | GPLv3 | None | Yes | Yes | No | Yes | 1 | 2019 | https://ieeexplore.ieee.org/abstract/document/8593603 |
| [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) ![VINS-Mono](https://img.shields.io/github/stars/HKUST-Aerial-Robotics/VINS-Mono.svg)| Yes | GPLv3 | None | Yes | Yes | No | No | 1 | 2017 | https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert | 
| [OpenVINS](https://github.com/rpng/open_vins) ![OpenVINS](https://img.shields.io/github/stars/rpng/open_vins.svg)| Yes | GPLv3 | None | Yes | Yes | No | N/A | 1 | 2019 | https://udel.edu/~ghuang/iros19-vins-workshop/papers/06.pdf | 
| [SE2SLAM](https://github.com/izhengfan/se2lam) ![SE2SLAM](https://img.shields.io/github/stars/izhengfan/se2lam.svg)| Yes | MIT | geometry_msgs::Pose / geometry_msgs::PoseStamped | Yes | Yes | No | Yes | 1 | 2019 | https://fzheng.me/icra/2019.pdf | 
| [SE2CLAM](https://github.com/izhengfan/se2clam) ![SE2CLAM](https://img.shields.io/github/stars/izhengfan/se2clam.svg)| Yes | MIT | geometry_msgs::PoseStamped | Yes | Yes | No | Yes | 1 | 2018 |  https://ieeexplore.ieee.org/document/8357438 | 
| [VINS-FusionGPU](https://github.com/pjrambo/VINS-Fusion-gpu) ![VINS-FusionGPU](https://img.shields.io/github/stars/pjrambo/VINS-Fusion-gpu.svg)| No | GPLv3 |
| [DSO](https://github.com/JakobEngel/dso_ros) ![DSO](https://img.shields.io/github/stars/JakobEngel/dso_ros.svg)| No | GPLv3 | None | Yes | Yes | No | No | No | 1 | 2016 | https://vision.in.tum.de/_media/spezial/bib/engel2016dso.pdf | 
| [DSO Ros2](https://github.com/goktug97/dso_ros2) | Yes | GPLv3 |
|[Edge Direct VO](https://github.com/kevinchristensen1/EdgeDirectVO) | Yes | N/A | N/A | Yes | No | No | No | None | 2019 | https://arxiv.org/abs/1906.04838 | N/A |
| [Maplab](https://github.com/ethz-asl/maplab) ![Maplab](https://img.shields.io/github/stars/ethz-asl/maplab.svg) | Yes | GPLv3 | N/A | Yes | Yes | No | Yes | 1 | 2018 | https://arxiv.org/abs/1711.10250 |
| [ORB_SLAM2 Ros2](https://github.com/alsora/ORB_SLAM2) ![ORB_SLAM2 Ros2](https://img.shields.io/github/stars/alsora/ORB_SLAM2.svg)| No | GPLv3 | visualization_msgs::msg::Marker |
| [SVO](https://github.com/uzh-rpg/rpg_svo) ![SVO](https://img.shields.io/github/stars/uzh-rpg/rpg_svo.svg)| No | GPLv3 | 
| [SVO 2.0](https://github.com/uzh-rpg/rpg_svo_example) | No | N/A (GPLv3?) |
| [ROS Mono VO](https://github.com/atomoclast/ros_mono_vo) ![ROS Mono VO](https://img.shields.io/github/stars/atomoclast/ros_mono_vo.svg)| No | BSD-2 |
| [SIVO](https://github.com/navganti/SIVO) ![SIVO](https://img.shields.io/github/stars/navganti/SIVO.svg)| Yes | GPLv3 |
| [dslam open](https://github.com/uzh-rpg/dslam_open) | Yes | GPLv3 |
| [LIMO](https://github.com/johannes-graeter/limo) ![LIMO](https://img.shields.io/github/stars/johannes-graeter/limo.svg)| Yes | GPLv3 |
| [Stereo DSO](https://github.com/HorizonAD/stereo_dso) ![Stereo DSO](https://img.shields.io/github/stars/HorizonAD/stereo_dso.svg)| No | GPLv3 |
| [VISO2 Python](https://github.com/AtlasBuggy/libviso2-python) ![VISO2 Python](https://img.shields.io/github/stars/AtlasBuggy/libviso2-python.svg)| No | GPLv3 inflicted |
| [MonoVO Python](https://github.com/uoip/monoVO-python) ![MonoVO Python](https://img.shields.io/github/stars/uoip/monoVO-python.svg)| No | N/A
| [DPPTAM](https://github.com/alejocb/dpptam) | No | GPLv3 |
| [StVO-PL](https://github.com/rubengooj/StVO-PL) ![StVO-PL](https://img.shields.io/github/stars/rubengooj/StVO-PL.svg)| Yes | GPLv3 |
| [PL-SLAM](https://github.com/rubengooj/pl-slam) ![PL-SLAM](https://img.shields.io/github/stars/rubengooj/pl-slam.svg) | Yes | GPLv3 |
| [MSCKF_VIO](https://github.com/KumarRobotics/msckf_vio) ![MSCKF_VIO](https://img.shields.io/github/stars/KumarRobotics/msckf_vio.svg)| Yes | GPLv3/CC BY-NC 3.0-like |
| [REBiVO](https://github.com/JuanTarrio/rebvo) [REBiVO](https://img.shields.io/github/stars/JuanTarrio/rebvo.svg) | No | GPLv3 |
| [R-VIO](https://github.com/rpng/R-VIO.git)



#### Additional information
**Active** is used loosely and subjectively, pretty much if I see that either some PR has been merged, commit has been made or even some maintainer/author has addressed an issue the past 9 - 12 month I consider it active.

Some libraries are addded that do not have a ROS-wrapper, however they might be there because it would be fairly straightforward to write a wrapper, or because the implementation is interesting for inspiration or similar.

I'm mostly interested in ROS2, so I do not break down the version on different release, neither for ROS1 or ROS2, but I see that that might be valuable as well.

### VO tools and calibration
| Package  |  Active | Topics published |  Stereo | Mono  |  Omni | SLAM | ROS version | Year | Paper |  License | 
|---|---|---|---|---|---|---|---|---|---|---|
|[VIO Data simulation](https://github.com/HeYijia/vio_data_simulation) | Yes | N/A | N/A | N/A | N/A | N/A | N/A | N/A |
| [VSLAM evaluation](https://github.com/nicolov/vslam_evaluation) |
| [vicalib](https://github.com/arpg/vicalib) | No
| https://github.com/vislearn/LessMore |
| [kalibr](https://github.com/ethz-asl/kalibr) |
| [Time Autosync](https://github.com/ethz-asl/time_autosync)
| [CRISP](https://github.com/hovren/crisp) | No | GPLv3


### Other VO resources
https://github.com/gaoxiang12/slambook
https://github.com/avisingh599/mono-vo
https://github.com/AtsushiSakai/PythonRobotics
https://github.com/tzutalin/awesome-visual-slam
https://github.com/marknabil/SFM-Visual-SLAM