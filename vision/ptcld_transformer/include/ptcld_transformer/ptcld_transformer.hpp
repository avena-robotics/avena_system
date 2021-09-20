#ifndef PTCLD_TRANSFORMER_COMPONENT_HPP
#define PTCLD_TRANSFORMER_COMPONENT_HPP

// ___CPP___
#include <memory>
#include <functional>
#include <nlohmann/json.hpp>

// ___PCL___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

// ___ROS2___
#include "rclcpp/rclcpp.hpp"

// ___Avena___
#include "ptcld_transformer/visibility_control.h"


#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"
#include <pcl/filters/crop_box.h>
#include "custom_interfaces/srv/data_store_rgbd_sync_select.hpp"

using namespace std::chrono_literals;


namespace ptcld_transformer
{
  using json = nlohmann::json;
  // struct CamerasData
  // {
  //   CamerasData() : cam1_ptcld(new pcl::PointCloud<pcl::PointXYZ>), cam2_ptcld(new pcl::PointCloud<pcl::PointXYZ>) {}
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cam1_ptcld;
  //   pcl::PointCloud<pcl::PointXYZ>::Ptr cam2_ptcld;

  //   using SharedPtr = std::shared_ptr<CamerasData>;
  // };

  struct CameraTransform
  {
    Eigen::Affine3f cam_aff;

    using SharedPtr = std::shared_ptr<CameraTransform>;
  };

  struct TransformedPointClouds
  {
    TransformedPointClouds()
        : cam1_ptcld_trans(new pcl::PointCloud<pcl::PointXYZ>), cam2_ptcld_trans(new pcl::PointCloud<pcl::PointXYZ>), merged_ptcld(new pcl::PointCloud<pcl::PointXYZ>) {}
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam1_ptcld_trans;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cam2_ptcld_trans;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_ptcld;

    using SharedPtr = std::shared_ptr<TransformedPointClouds>;
  };

  struct TableArea
  {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
  };

  class PtcldTransformer 
  {
  public:
    COMPOSITION_PUBLIC
    PtcldTransformer(rclcpp::Node *node, bool &transformer_status);
    ~PtcldTransformer();
    pcl::PointCloud<pcl::PointXYZ>::Ptr transfromPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld);

  private:
    rclcpp::Client<custom_interfaces::srv::DataStoreRgbdSyncSelect>::SharedPtr _rgbd_sync_client;

    // CameraTransform::SharedPtr _cameras_parameters;
    std::map<std::string, CameraTransform::SharedPtr> _cam_transform;
    size_t _cameras_amount;
    TableArea _table_area;
    rclcpp::Node *_node;


    void _convertCloudToPCL(const sensor_msgs::msg::PointCloud2 &ros_cloud_msg,  pcl::PointCloud<pcl::PointXYZ>::Ptr &out_pcl_cloud);
    void _convertCloudToRos(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud, sensor_msgs::msg::PointCloud2 &out_ros_cloud_msg );

    bool  _getCamerasTransform();
    pcl::PointCloud<pcl::PointXYZ>::Ptr _processCamerasData(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld, CameraTransform::SharedPtr cameras_parameters);
    void _filterAndTransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, const Eigen::Affine3f &camera_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud);
    void _getTableAreaFromParametersServer();
    builtin_interfaces::msg::Time _getMergedPtcldTime(const builtin_interfaces::msg::Time &cam1_stamp, const builtin_interfaces::msg::Time &cam2_stamp);

    rclcpp::TimerBase::SharedPtr _timer;
    bool _param_server_read;


  };

} // namespace ptcld_transformer

#endif
