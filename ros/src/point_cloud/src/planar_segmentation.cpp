
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ros/console.h>

ros::Publisher pub;

float distance_threshold;       //: 0.25    (done)
int max_iterations;             //: 1000    (done)
bool optimize_coefficients;     //: true    (done)
float normal_distance_weight;   //: 0.1
float eps_angle;                //: 0.27    (done)


void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& input)
{

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZ>);

    // Create a container for the data.
    // Convert PointCloud2 msg to PCL
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);

    pcl::fromPCLPointCloud2(pcl_pc2,*cloud_filtered);

    // perform the calculation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setEpsAngle(eps_angle);
    seg.setMaxIterations (max_iterations);
    seg.setDistanceThreshold (distance_threshold);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int nr_points = (int) cloud_filtered->points.size ();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);

        // Extract the inliers
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);

        pcl::concatenateFields (*cloud_output, *cloud_p, *cloud_output);

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_filtered.swap (cloud_f);
    }

    // publish filtered cloud data
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_output, cloud_publish);
    cloud_publish.header = input->header;

    pub.publish(cloud_publish);
}


int main (int argc, char** argv){

    // Initialize ROS
    ros::init (argc, argv, "planar_segmentation");
    ros::NodeHandle nh;

//    nh.getParam("distance_threshold", distance_threshold);
//    nh.getParam("max_iterations", max_iterations);
//    nh.getParam("optimize_coefficients", optimize_coefficients);
//    nh.getParam("normal_distance_weight", normal_distance_weight);
//    nh.getParam("eps_angle", eps_angle);

    distance_threshold = 0.20;
    max_iterations = 1000;
    optimize_coefficients = true;
    normal_distance_weight = 0.1;
    eps_angle = 30.0f * (M_PI/180.0f);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/voxel_grid/output", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/planar_segmentation/output", 1);

    // Spin
    ros::spin ();

    return 0;
}


