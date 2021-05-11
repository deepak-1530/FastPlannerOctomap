/**
 * Adding gaussian noise to Pointcloud obtained from ROS
**/
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/filters/voxel_grid.h>
#include<random> // for random no. generation

ros::Publisher pub;

float var;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  // convert to pcl point cloud and then add gaussian random noise
  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::fromROSMsg(*cloud_msg, *xyz_cloud);  

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ> ());
  xyz_cloud_filtered->points.resize(xyz_cloud->points.size());
  xyz_cloud_filtered->header = xyz_cloud->header;
  xyz_cloud_filtered->width  = xyz_cloud->width;
  xyz_cloud_filtered->height = xyz_cloud->height;

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, var); // mean and standard deviation

  for(size_t points_i=0; points_i<xyz_cloud->points.size(); ++points_i)
  {
      xyz_cloud_filtered->points[points_i].x = xyz_cloud->points[points_i].x + static_cast<float>(distribution(generator));
      xyz_cloud_filtered->points[points_i].y = xyz_cloud->points[points_i].y + static_cast<float>(distribution(generator));
      xyz_cloud_filtered->points[points_i].z = xyz_cloud->points[points_i].z + static_cast<float>(distribution(generator));
  }
  
  sensor_msgs::PointCloud2 input_xyz_filtered;
  pcl::toROSMsg(*xyz_cloud_filtered, input_xyz_filtered);
  pub.publish (input_xyz_filtered);
}

int main(int argc, char** argv)
{
    std::cout<<"Enter variance ";
    //std::cin>>var;
    ros::init(argc, argv, "pcl_basics");
    ros::NodeHandle nh;
    nh.getParam("pcNoise/noise", var);
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points",1, cloud_cb);
    pub = nh.advertise<sensor_msgs::PointCloud2>("output",1);
    ros::spin();
    return 0;
}