#include "pcl_converter.h"

PCLConverter::PCLConverter() : Node("pcl_converter"), ready{false} {

    point_cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/depth_camera/points",
        10,
        std::bind(&PCLConverter::point_cloud2_callback, this, std::placeholders::_1)
    );
}

void PCLConverter::point_cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    fromROSMsg(*msg, *pcl_cloud);

    if(!ready) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr good_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        RCLCPP_INFO(this->get_logger(), "wchodze");

        /*for(size_t i=0; i<pcl_cloud->points.size(); ++i) {

            if(std::isfinite(pcl_cloud->points[i].x) && std::isfinite(pcl_cloud->points[i].y) && std::isfinite(pcl_cloud->points[i].z)) {
                if(std::abs(pcl_cloud->points[i].y)>0.0125 && pcl_cloud->points[i].z<2.9) {
                    good_cloud->points.push_back(pcl_cloud->points[i]);
                }
            }

            if(i%1000==0) {
                RCLCPP_INFO(this->get_logger(), "progress %lu", i);
            }
        }

        good_cloud->height = 1;
        good_cloud->width = good_cloud->points.size();

        RCLCPP_INFO(this->get_logger(), "saving PCD file...");
        pcl::io::savePCDFileBinary("hand_bin.pcd", *good_cloud);
        pcl::io::savePCDFileASCII("hand_ascii.pcd", *good_cloud);
        RCLCPP_INFO(this->get_logger(), "PCD file saved");*/

        /*for(size_t i=0; i<pcl_cloud->points.size(); ++i) {

            if(std::isfinite(pcl_cloud->points[i].x) && std::isfinite(pcl_cloud->points[i].y) && std::isfinite(pcl_cloud->points[i].z)) {
                good_cloud->points.push_back(pcl_cloud->points[i]);
            }
        }

        good_cloud->height = 1;
        good_cloud->width = good_cloud->points.size();*/

        RCLCPP_INFO(this->get_logger(), "saving PCD file...");
        pcl::io::savePCDFileBinary("scene_3_hands_bin.pcd", *pcl_cloud);
        RCLCPP_INFO(this->get_logger(), "PCD file saved");

        ready = true;
    }

}
