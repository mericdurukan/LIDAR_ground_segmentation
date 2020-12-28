#include <ros/ros.h>
#include "ground_segmentation.h"





int num_iter=4; //number of iteration to estimate plane
int num_lpr=100; //number of seed points
float th_seeds=0.01; // The threshold for seeds
float th_dist=0.01; // The confidence threshold for the test ground plane
float initial_height = 0.45;// Initial sensor height from the ground plane



ros::Publisher pub; //Ros publisher


void ground_ex (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud


    pcl::PointCloud<pcl::PointXYZ> lidar_in; //input pointcloud for the ground segmentation.
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());  //ground point cloud to be taken
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>());  //non_ground_cloud to be taken

    pcl::fromROSMsg (*input, lidar_in); // This function converts ros' PointCloud2 object to PCL pointcloud type

    ground_seg* ground_segmentation = new ground_seg(num_iter,num_lpr,th_seeds, th_dist, initial_height); //Create the object from the class.
    ground_segmentation->segment(lidar_in, ground_cloud, non_ground_cloud); //segment the given cloud

    delete ground_segmentation;//Delete the object

    sensor_msgs::PointCloud2::Ptr ground_segmented(new sensor_msgs::PointCloud2 ()); //Create a new message to publish the result. The message type is PointCloud2
    pcl::toROSMsg (*non_ground_cloud, *ground_segmented); //This function converts the pcl PointCLoud to ros's pointcloud
    ground_segmented->header.stamp = input->header.stamp; // optional for seeing the point cloud in rviz
    ground_segmented->header.frame_id = input->header.frame_id;// essential for seeing the point cloud in rviz

    pub.publish (ground_segmented); //publish the results


    //Information:
    cout<<"non ground: "<<non_ground_cloud->size()<<endl;
    cout<<"ground: "<<ground_cloud->size()<<endl;

}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ground_segmentation");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/your_lidar_topic", 1, ground_ex);


    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("ground_segmentation", 1);

    // Spin
    ros::spin ();
}
