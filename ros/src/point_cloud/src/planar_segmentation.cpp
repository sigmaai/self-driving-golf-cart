
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
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
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    // perform calculations
    // --------------------
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (optimize_coefficients);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (distance_threshold);
    seg.setEpsAngle(eps_angle);
    seg.setMaxIterations(max_iterations);

// Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) temp_cloud->points.size ();
    // While 30% of the original cloud is still there
    while (temp_cloud->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (temp_cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(temp_cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloudF;
        extract.filter(cloudF);
        temp_cloud->swap(cloudF);

        i++;
    }

    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*temp_cloud,cloud_publish);
    cloud_publish.header = input->header;

    // std::cerr << "Point cloud data: " << cloud_msg->points.size () << " points" << std::endl;

    pub.publish(cloud_publish);
}

int main (int argc, char** argv){

    ROS_DEBUG("Hello %s", "World");

    // Initialize ROS
    ros::init (argc, argv, "planar_segmentation");
    ros::NodeHandle nh;

//    nh.getParam("distance_threshold", distance_threshold);
//    nh.getParam("max_iterations", max_iterations);
//    nh.getParam("optimize_coefficients", optimize_coefficients);
//    nh.getParam("normal_distance_weight", normal_distance_weight);
//    nh.getParam("eps_angle", eps_angle);

    distance_threshold = 0.25;
    max_iterations = 1000;
    optimize_coefficients = true;
    normal_distance_weight = 0.1;
    eps_angle = 0.27;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/voxel_grid/output", 1, cloud_callback);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/planar_segmentation/output", 1);

    // Spin
    ros::spin ();

    return 0;
}


