

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class PCLConverter : public rclcpp::Node
{
public:
  PCLConverter();
 

private:
  void point_cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_sub_;
};


using std::placeholders::_1;


PCLConverter::PCLConverter() : Node("pcl_converter")
{
  RCLCPP_INFO(this->get_logger(), "2137");
  point_cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/depth_camera/points",
    10,
    std::bind(&PCLConverter::point_cloud2_callback, this, std::placeholders::_1));
    
}


void PCLConverter::point_cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
 RCLCPP_INFO(this->get_logger(), "Received PointCloud2 message");

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  fromROSMsg(*msg, *pcl_cloud);
  
  if(msg->width == pcl_cloud->width){
      RCLCPP_INFO(this->get_logger(), "ok");

  }else{
     RCLCPP_INFO(this->get_logger(), "nok");

  }

  
  

  RCLCPP_INFO(this->get_logger(), "PointCloud2 message converted to PCL format");
}



int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PCLConverter>());

  rclcpp::shutdown();
  return 0;
}