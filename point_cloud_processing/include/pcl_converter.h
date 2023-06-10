#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>

class PCLConverter : public rclcpp::Node {
    using PointType = pcl::PointXYZ;
    using NormalType = pcl::Normal;
    using RFType = pcl::ReferenceFrame;
    using DescriptorType = pcl::SHOT352;

    static constexpr float model_ss_ = 0.005f;
    static constexpr float scene_ss_ = 0.01f;
    static constexpr float rf_rad_ = 0.015f;
    static constexpr float descr_rad_ = 0.02f;
    static constexpr float cg_size_ = 0.01f;
    static constexpr float cg_thresh_ = 5.0f;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_sub_;
    bool busy;
    pcl::PointCloud<PointType>::Ptr model;

    struct Transform {
        Eigen::Matrix3f rotation;
        Eigen::Vector3f translation;
    };

    void save_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name);
    void point_cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    static std::vector<Transform> recognize(pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr scene);

public:
    PCLConverter();
};
