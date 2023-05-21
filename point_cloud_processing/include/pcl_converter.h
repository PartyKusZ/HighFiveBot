#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

class PCLConverter : public rclcpp::Node {
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_sub_;

    void point_cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

public:
    PCLConverter();
};
