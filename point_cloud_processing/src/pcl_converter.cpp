#include "pcl_converter.h"

PCLConverter::PCLConverter() : Node("pcl_converter"), busy{false}, model(new pcl::PointCloud<PointType>()), ready{false} {

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    pcl::io::loadPCDFile("src/pcl/hand_bin.pcd", *model);

    Eigen::Vector3f hand_center;

    hand_center(0) = 0;
    hand_center(1) = 0;
    hand_center(2) = 0;

    for(size_t i=0; i<model->points.size(); ++i) {
        hand_center(0) +=model->points[i].x;
        hand_center(1) +=model->points[i].y;
        hand_center(2) +=model->points[i].z;
    }

    hand_center(0) /=model->points.size();
    hand_center(1) /=model->points.size();
    hand_center(2) /=model->points.size();

    for(size_t i=0; i<model->points.size(); ++i) {
        model->points[i].x -=hand_center(0);
        model->points[i].y -=hand_center(1);
        model->points[i].z -=hand_center(2);
    }

    point_cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/depth_camera/points",
        10,
        std::bind(&PCLConverter::point_cloud2_callback, this, std::placeholders::_1)
    );

    using namespace std::chrono_literals;

    timer = this->create_wall_timer(100ms, std::bind(&PCLConverter::timer_callback, this));
}

void PCLConverter::save_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name) {
    if(!busy) {
        busy = true;

        //RCLCPP_INFO(this->get_logger(), "wchodze");

        /*pcl::PointCloud<pcl::PointXYZ>::Ptr good_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for(size_t i=0; i<pcl_cloud->points.size(); ++i) {

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
        pcl::io::savePCDFileBinary(name, *cloud);
        RCLCPP_INFO(this->get_logger(), "PCD saved");

        busy = false;
    }
}

void PCLConverter::compute(pcl::PointCloud<PointType>::Ptr scene) {
    busy = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr good_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for(size_t i=0; i<scene->points.size(); ++i) {
        if(scene->points[i].y<0.2 && scene->points[i].z<1) {
            good_cloud->points.push_back(scene->points[i]);
        }
    }

    good_cloud->height = 1;
    good_cloud->width = good_cloud->points.size();

    const std::vector<Transform> transforms = recognize(model, good_cloud);

    busy = false;

    std::cout << "Model instances found: " << transforms.size () << std::endl;
    for (std::size_t i = 0; i < transforms.size (); ++i) {
        std::cout << "\n    Instance " << i + 1 << ":" << std::endl;

        Eigen::Matrix3f rotation = transforms[i].rotation;
        Eigen::Vector3f translation = transforms[i].translation;

        RCLCPP_INFO(this->get_logger(), "            | %6.3f %6.3f %6.3f |", rotation (0,0), rotation (0,1), rotation (0,2));
        RCLCPP_INFO(this->get_logger(), "        R = | %6.3f %6.3f %6.3f |", rotation (1,0), rotation (1,1), rotation (1,2));
        RCLCPP_INFO(this->get_logger(), "            | %6.3f %6.3f %6.3f |", rotation (2,0), rotation (2,1), rotation (2,2));
        RCLCPP_INFO(this->get_logger(), "\n");
        RCLCPP_INFO(this->get_logger(), "        t = < %0.3f, %0.3f, %0.3f >", translation (0), translation (1), translation (2));
    }

    if(transforms.size()==0) {
        return;
    }

    hand = *std::min(transforms.begin(), transforms.end());
    ready = true;
}

void PCLConverter::timer_callback() {
    //if(!ready) {
    //    return;
    //}

    ready = false;

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "camera_link";
    t.child_frame_id = "hand";

    t.transform.translation.x = +hand.translation(2);
    t.transform.translation.y = -hand.translation(0);
    t.transform.translation.z = -hand.translation(1);

    tf2::Matrix3x3 rot;
    rot.setValue(
        hand.rotation(0, 0),
        hand.rotation(0, 1),
        hand.rotation(0, 2),
        hand.rotation(1, 0),
        hand.rotation(1, 1),
        hand.rotation(1, 2),
        hand.rotation(2, 0),
        hand.rotation(2, 1),
        hand.rotation(2, 2)
    );

    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);

    tf2::Quaternion q;
    q.setRPY(yaw, 0, pitch);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

void PCLConverter::point_cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    fromROSMsg(*msg, *pcl_cloud);

    if(busy) {
        return;
    }

    std::thread thread(std::bind(&PCLConverter::compute, this, std::placeholders::_1), pcl_cloud);

    thread.detach();
}

std::vector<PCLConverter::Transform> PCLConverter::recognize(pcl::PointCloud<PointType>::Ptr model, pcl::PointCloud<PointType>::Ptr scene) {

  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);

  //
  //  Downsample Clouds to Extract keypoints
  //

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_ss_);
  uniform_sampling.filter (*model_keypoints);
  //std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  uniform_sampling.filter (*scene_keypoints);
  //std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (std::size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  //std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //
  //  Compute (Keypoints) Reference Frames only for Hough
  //
  pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
  pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

  pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
  rf_est.setFindHoles (true);
  rf_est.setRadiusSearch (rf_rad_);

  rf_est.setInputCloud (model_keypoints);
  rf_est.setInputNormals (model_normals);
  rf_est.setSearchSurface (model);
  rf_est.compute (*model_rf);

  rf_est.setInputCloud (scene_keypoints);
  rf_est.setInputNormals (scene_normals);
  rf_est.setSearchSurface (scene);
  rf_est.compute (*scene_rf);

  //  Clustering
  pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
  clusterer.setHoughBinSize (cg_size_);
  clusterer.setHoughThreshold (cg_thresh_);
  clusterer.setUseInterpolation (true);
  clusterer.setUseDistanceWeight (false);

  clusterer.setInputCloud (model_keypoints);
  clusterer.setInputRf (model_rf);
  clusterer.setSceneCloud (scene_keypoints);
  clusterer.setSceneRf (scene_rf);
  clusterer.setModelSceneCorrespondences (model_scene_corrs);

  //clusterer.cluster (clustered_corrs);
  clusterer.recognize (rototranslations, clustered_corrs);

  std::vector<Transform> transformations;

  for(std::size_t i = 0; i < rototranslations.size (); ++i) {
    Transform transformation;
    transformation.rotation = rototranslations[i].block<3,3>(0, 0);
    transformation.translation = rototranslations[i].block<3,1>(0, 3);
    transformations.push_back(transformation);
  }

  return transformations;
}
