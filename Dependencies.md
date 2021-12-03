<!--
 * @Author: kinggreat24
 * @Date: 2021-05-05 21:49:55
 * @LastEditTime: 2021-12-03 20:10:05
 * @LastEditors: kinggreat24
 * @Description: 
 * @FilePath: /d2vl_slam/Dependencies.md
 * 可以输入预定的版权声明、个性签名、空行等
-->
##List of Known Dependencies
###DV-LOAM version 1.0

In this document we list all the pieces of code included  by ORB-SLAM2 and linked libraries which are not property of the authors of ORB-SLAM2.


#####Code in **orb_slam2** **src** and **include** folders

* *ORBextractor.cc*.
This is a modified version of orb.cpp of OpenCV library. The original code is BSD licensed.

* *PnPsolver.h, PnPsolver.cc*.
This is a modified version of the epnp.h and epnp.cc of Vincent Lepetit.
This code can be found in popular BSD licensed computer vision libraries as [OpenCV](https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/epnp.cpp) and [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp). The original code is FreeBSD.

* Function *ORBmatcher::DescriptorDistance* in *ORBmatcher.cc*.
The code is from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel.
The code is in the public domain.

#####Code in Thirdparty folder

* All code in **DBoW2** folder.
This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are BSD licensed.

* All code in **g2o** folder.
This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are BSD licensed.

#####Library dependencies

* **OpenCV-3.2**.
BSD license.

* **Eigen3**.
For versions greater than 3.1.1 is MPL2, earlier versions are LGPLv3.

* **G2O**.

* **Ceres-Solver**.

* **PCL-1.7.0**.

* **Sophus**.

* **vikit-common**.

* **ROS**.
BSD license. In the manifest.xml the only declared package dependencies are roscpp, tf, sensor_msgs, image_transport, cv_bridge, which are all BSD licensed.
