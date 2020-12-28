/* The codes are written by: Meric Durukan
 * for Bogazici University Intelligent Systems Laboratory
 * Date: 12/28/2020
 * If the code includes a bug or bugs please inform me: mericdurukan at gmail.com
 */

#include "ground_segmentation.h"


// The function does not belong to the class.
// The function compares the "z" values from the point cloud.
// It is necessary to take the lowest height.
bool compare_z(pcl::PointXYZ point1, pcl::PointXYZ point2){
    return point1.z<point2.z; //comparision
}



//The class has one constructor.
//The constructor takes the parameter for the ground segmentation.


ground_seg::ground_seg(int in_num_iter, int in_num_lpr, float in_th_seeds, float in_th_dist, float in_height)
{

    this->num_iter=in_num_iter; //number of iteration to estimate plane
    this->num_lpr=in_num_lpr; //number of seed points
    this->th_seeds=in_th_seeds; // The threshold for seeds
    this->th_dist=in_th_dist; // The confidence threshold for the test ground plane
    this->initial_height = in_height; // Initial sensor height from the ground plane

}

// The destructor for the class
ground_seg::~ground_seg()
{

}



// The function is called by the user.
// The first parameter is the input point cloud
// The second parameter is ground cloud(please send an empty point cloud)
// The third parameter is non-ground cloud(please send an empty point cloud)

void ground_seg::segment(pcl::PointCloud<pcl::PointXYZ> in_cloud , pcl::PointCloud<pcl::PointXYZ>::Ptr  &ground_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr  &not_ground_cloud)
{


    pcl::PointCloud<pcl::PointXYZ>::Ptr g_seeds_pc(new pcl::PointCloud<pcl::PointXYZ>()); // The cloud for seed points
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_ground_pc(new pcl::PointCloud<pcl::PointXYZ>()); // The cloud for the points from the ground
    pcl::PointCloud<pcl::PointXYZ>::Ptr g_not_ground_pc(new pcl::PointCloud<pcl::PointXYZ>()); //The cloud for the points from ground extracted points.


    pcl::PointCloud<pcl::PointXYZ> all_points_cloud; // The cloud for the filtered point cloud
    pcl::PointCloud<pcl::PointXYZ> not_sorted_cloud; // The cloud for the take not sorted cloud

    //Filtering

    for(int ite = 0; ite<in_cloud.size(); ite++){

        if(in_cloud.points[ite].z > initial_height * this->filter_constant ) //Filter the cloud according to initial height.
            all_points_cloud.points.push_back(in_cloud.points[ite]); //If the point is higher than the this filter  parameter, it is taken for the ground extraction.
    }




    not_sorted_cloud = all_points_cloud; //Take the orijinal(not sorted cloud)

    std::sort(all_points_cloud.points.begin(),all_points_cloud.end(),compare_z); //sort the point cloud according to z coordinates in ascending order.
    extract_initial_seeds(all_points_cloud, g_seeds_pc); //We need initial points to find initial plane. This function extracts initial points from the ground plane.

    g_ground_pc = g_seeds_pc; //The first seeds are a base for the ground points.


    //The iteration to find more points for the ground.
    for(int i=0;i<this->num_iter;i++){


        float ground_threshold = estimate_plane(g_ground_pc); //This function returns with the threshold distance for the Z distance.
        g_ground_pc->clear();
        g_not_ground_pc->clear();


        float result=0; //A variable for the distance between the point and the ground surface.



        for(int r=0;r<all_points_cloud.size();r++){

            result =not_sorted_cloud.points[r].x * normal(0) +  not_sorted_cloud.points[r].y * normal(1) + not_sorted_cloud.points[r].z* normal(2) ; // How the point far the ground plane.

            //If the point is far from the ground plane, the point is evaluated as a non ground point, otherwise the points is a ground point.
            if(result<=ground_threshold)
            {


                g_ground_pc->points.push_back(not_sorted_cloud[r]); //Take the ground points

            }
            else
            {
                g_not_ground_pc->points.push_back(not_sorted_cloud[r]); // Take the non-ground points.
            }
        }

    }



    ground_cloud->points.resize(g_ground_pc->size()); // Output cloud for the ground points.
    not_ground_cloud->points.resize(g_not_ground_pc->size()); // Output cloud for non ground points


    //Two "for"s aim to take the two clouds.
    for(size_t ground_it = 0; ground_it < g_ground_pc->size(); ground_it++)
        ground_cloud->points[ground_it] = g_ground_pc->points[ground_it];

    for(size_t non_ground_it = 0; non_ground_it < g_not_ground_pc->size(); non_ground_it++ )
        not_ground_cloud->points[non_ground_it] = g_not_ground_pc->points[non_ground_it];


}






float ground_seg::estimate_plane(pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_points)
{

    Eigen::Matrix3f cov; //The covariance matrix for the estimation ground surface normal
    Eigen::Vector4f pc_mean; //The mean vector for the ground plane



    pcl::computeMeanAndCovarianceMatrix(*ground_points,cov,pc_mean); //Compute mean and covariance matrix


    // !!! In our lab, some users claim this function does not create eigenvalues in descending order. However, in my experiments the function works correctly. If your eigenvalues are not in descending order, please change the function.  !!!
    JacobiSVD<MatrixXf> svd(cov, Eigen::ComputeFullU); //Singular value decomposition to find eigenvalues and eigenvectors.



    //use the least singular vector as normal

    normal=(svd.matrixU().col(2)); //Normal vector(that correspons to the lowest eigenvalue.)

    Eigen::Vector3f seeds_mean = pc_mean.head<3>(); //Mean value for the ground points.



    float d=(normal.transpose()*seeds_mean)(0,0); //The distance to evaluate the ground plane.


    float  th_dist_d=th_dist+d; // This line creates a threshold to question the distance in z axis.

    return th_dist_d; //the line returns the threshold from the function.

}





//The function extracts the initial seeds from the ground.
//The first parameter is the filtered and sorted point cloud.
//The second parameter is the ground seeds(the parameter is an output. Please give an empty cloud.)
void ground_seg::extract_initial_seeds(pcl::PointCloud<pcl::PointXYZ> p_sorted,pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_seeds)
{



    float sum = 0.0; // The variable is to take the sum of the points whose size is given by num_lpr parameter


    //Summation for seed points.

    for(int i=0; i< num_lpr ; i++){

        sum = sum + p_sorted.points[i].z; //Sum all seed points.

    }



    float lpr_height = sum / num_lpr; //averaging, lpr_height is an average for the seed points


    // We found the average for the given number of points.
    // We want to take more points from the lpr_height.
    // To decide the height for initial seeds, we use both lpr_height and th_seeds.

    for(int k=0;k<p_sorted.size();k++){

        if(p_sorted.points[k].z < lpr_height +th_seeds ) //compare the threshold height for the seeds
        {

            ground_seeds->points.push_back(p_sorted.points[k]); // These are the initial ground points.

        }

    }


}
