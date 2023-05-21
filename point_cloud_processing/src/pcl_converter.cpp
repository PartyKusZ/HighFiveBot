#include "pcl_converter.h"

PCLConverter::PCLConverter() : Node("pcl_converter") {

    point_cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/depth_camera/points",
        10,
        std::bind(&PCLConverter::point_cloud2_callback, this, std::placeholders::_1)
    );
}

void PCLConverter::point_cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    fromROSMsg(*msg, *pcl_cloud);

    if(msg->width == pcl_cloud->width) {
        RCLCPP_INFO(this->get_logger(), "ok xddd");
    } else {
        RCLCPP_INFO(this->get_logger(), "nok");
    }
}
