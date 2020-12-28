/* The codes are written by: Meric Durukan
 * for Bogazici University Intelligent Systems Laboratory
 * Date: 12/28/2020
 * The strategy for the ground estimation is an implementation of this work:
 * @inproceedings{zermas2017fast,
  title={Fast segmentation of 3d point clouds: A paradigm on lidar data for autonomous vehicle applications},
  author={Zermas, Dimitris and Izzat, Izzat and Papanikolopoulos, Nikolaos},
  booktitle={2017 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={5067--5073},
  year={2017},
  organization={IEEE}
}

 * We has slightly changed the work's strategy. We use the initial height to eliminate the noisly sensed points.
 *
 *
 * If the code includes a bug or bugs please inform me: mericdurukan at gmail.com
 *
 * I tested the impelentation on both my simulation for velodyne VLP-16 and Kitti tracking dataset
 * My github page includes the images from the scenes.
 *
 * The Usage:
 *
 * create a new class for the ground segmentation (initialization needs parameters: int in_num_iter, int in_num_lpr, float in_th_seeds,  float in_th_dist, float initial_height)
 * int num_iter; //number of iteration to estimate plane
 * int num_lpr; //number of seed points.
 * float th_seeds; // The threshold to decide initial seeds.
 * float th_dist; // The threshold distance to decide the point  belongs to whether the ground plane or not.
 * float initial_height; //The initial height for the sensor.
 * Then give your cloud to extract to ground.
 * The main function and my github page include detailed information and implementation.

*/




//C++ lib
#include <iostream>

//PCL lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

//From ROS env.
#include <velodyne_pointcloud/point_types.h>
#include <sensor_msgs/PointCloud2.h>


//Eigen lib.
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>



using namespace std;
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using namespace std;
using namespace pcl;




class ground_seg{


public : 

ground_seg(int in_num_iter, int in_num_lpr, float in_th_seeds,  float in_th_dist, float initial_height); //Constructor for the class.
~ground_seg();  //Destructor for the class
void segment(pcl::PointCloud<pcl::PointXYZ>  in_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr  &ground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &not_ground_cloud ); //The user calls this function to segment the ground.


private: 




int num_iter; //number of iteration to estimate plane
int num_lpr; //number of seed points
float th_seeds; // The threshold to decide initial seeds
float th_dist; // The threshold distance to decide the point  belongs to whether the ground plane or not.
float initial_height; // The initial sensor height to eliminate the noise.
float filter_constant = -1.5;  //To filter the noise from the observed points.
Eigen::VectorXf normal; //This vector contains the surface normal.(in z axis which is perpendicular to ground plane)


float estimate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_points);  
void extract_initial_seeds(pcl::PointCloud<pcl::PointXYZ> p_sorted,pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_seeds);


};


