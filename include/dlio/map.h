/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#ifndef DLIO_MAP_H
#define DLIO_MAP_H

#include "dlio/dlio.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "direct_lidar_inertial_odometry/srv/save_pcd.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp> // ROS 2 Service header

// PCL
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mutex>

class dlio::MapNode: public rclcpp::Node {

public:

  MapNode();
  ~MapNode();

  void start();

private:

  void getParams();

  void callbackKeyframe(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& keyframe);

  void savePCD(const std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Request> req,
               std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePCD::Response> res);

  // New Reset Map Service Callback
  void resetMap(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                std::shared_ptr<std_srvs::srv::Empty::Response> res);

  rclcpp::CallbackGroup::SharedPtr keyframe_cb_group;
  rclcpp::CallbackGroup::SharedPtr save_pcd_cb_group;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

  rclcpp::Service<direct_lidar_inertial_odometry::srv::SavePCD>::SharedPtr save_pcd_srv;
  
  // New Reset Map Service Member
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_map_srv;

  pcl::PointCloud<PointType>::Ptr dlio_map;
  pcl::VoxelGrid<PointType> voxelgrid;
  
  // Mutex for thread-safety during map clearing/updating
  std::mutex map_mutex;

  std::string odom_frame;

  double leaf_size_;

};

#endif