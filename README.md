# LIDAR_ground_segmentation

This repository is created to extract the ground from LIDAR scenes.

It is the implementation of the work:
```
@inproceedings{zermas2017fast,
  title={Fast segmentation of 3d point clouds: A paradigm on lidar data for autonomous vehicle applications},
  author={Zermas, Dimitris and Izzat, Izzat and Papanikolopoulos, Nikolaos},
  booktitle={2017 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={5067--5073},
  year={2017},
  organization={IEEE}
}
```
And, inspired repository: https://github.com/VincentCheungM/Run_based_segmentation


The repository is tested on Ubuntu 16.04, ROS-Kinetic



# Please follow the instructions to run the repo: 

Open a terminal 
Then, write: 
```
$ cd your_workspace/src/
$ git clone https://github.com/mericdurukan/LIDAR-ground-segmentation

```
# Important note: 

If you want to use the main file, Please change the main file's the topic of the subscriber.<br />
The topic is stated at ground_segmentation_main.cpp's line 56<br />
Please rewrite the topic.<br />

After that:

```
$ cd your_workspace/
$ catkin_make
$ rosrun LIDAR_ground_segmentation ground_segmentation
```

All code lines include detailed comments. 

# The usage:
-Add the header file into your code
-Create an object from the class:

```
ground_seg* ground_segmentation = new ground_seg(num_iter,num_lpr,th_seeds, th_dist, initial_height);
```
Then, segment the cloud:

```
pcl::PointCloud<pcl::PointXYZ> lidar_in; 
pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());  
pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>()); 
ground_segmentation->segment(lidar_in, ground_cloud, non_ground_cloud);
```
# The parameters: 

num_iter: The number of iteration to decide the ground cloud.<br />
num_lpr: The number of points that is used to find initial height.<br />
th_seeds: The threshold is to find the seed points.<br />
th_dist: The threshold is to find ground plane.<br />
initial_height: The height of the sensor to eliminate the noise. <br /> 

Please see the article to detail information about parameters. 


# RESULTS: 

# KITTI Tracking: 

![Alt text](https://github.com/mericdurukan/LIDAR_ground_segmentation/blob/main/images/kitti_seg.png)

Used parameters for kitti tracking:


num_iter: 5<br />
num_lpr: 10000<br />
th_seeds: 0.1<br />
th_dist: 0.3<br />
initial_height: 1.73<br />

Kitti is reachable via : http://www.cvlibs.net/datasets/kitti/

# Velodyne Simulator:  

![Alt text](https://github.com/mericdurukan/LIDAR_ground_segmentation/blob/main/images/ground_segmentation.png)
 
 
 
![Alt text](https://github.com/mericdurukan/LIDAR_ground_segmentation/blob/main/images/segmented_ground.png)
 
 
 Used parameters for velodyne simulation:


num_iter: 4<br />
num_lpr: 100<br />
th_seeds: 0.01<br />
th_dist: 0.01<br />
initial_height: 0.45<br />

Velodyne simulator is reachable via: http://wiki.ros.org/velodyne_simulator/
 

 contact info: mericdurukan@gmail.com
 
