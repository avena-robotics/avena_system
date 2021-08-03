#ifndef ESTIMATE_SHAPE_STRUCTURES_HPP
#define ESTIMATE_SHAPE_STRUCTURES_HPP

#include <opencv2/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nlohmann/json.hpp>
#include "estimate_shape_commons/logging_macros.hpp"

namespace estimate_shape
{
  using json = nlohmann::json;
  using Timer = helpers::Timer;

  struct CameraParameters
  {
    CameraParameters() = default;

    CameraParameters(const Eigen::Translation3f &trans, const Eigen::Quaternionf &orien, const std::string &cam_name)
        : camera_name(cam_name), translation(trans), orientation(orien)
    {
      affine = translation * orientation;
    }

    CameraParameters &operator=(const CameraParameters &other)
    {
      if (this == &other)
        return *this;
      camera_name = other.camera_name;
      translation = other.translation;
      orientation = other.orientation;
      affine = other.affine;
      return *this;
    }

    std::string camera_name;
    Eigen::Translation3f translation;
    Eigen::Quaternionf orientation;
    Eigen::Affine3f affine;
  };

  struct CamerasFrames
  {
    inline static const std::string camera_1 = "camera_1";
    inline static const std::string camera_2 = "camera_2";
  };

  struct Label
  {
    std::string label{};
    std::string fit_method{};
    json fit_method_parameters{};
    bool item = false;
    bool element = false;
    json components{};
    json raw_data{};
  };

  struct ItemElement
  {
    ItemElement()
        : element_mask_1(new cv::Mat),
          element_mask_2(new cv::Mat),
          element_depth_1(new cv::Mat),
          element_depth_2(new cv::Mat),
          // element_pcl_1(new pcl::PointCloud<pcl::PointXYZ>),
          // element_pcl_2(new pcl::PointCloud<pcl::PointXYZ>),
          element_pcl_1(new pcl::PointCloud<pcl::PointXYZRGB>),
          element_pcl_2(new pcl::PointCloud<pcl::PointXYZRGB>),
          pcl_merged(new pcl::PointCloud<pcl::PointXYZ>)
    {
    }
    int32_t id = -1;
    int32_t item_id = -1;
    std::string element_label{};
    std::shared_ptr<cv::Mat> element_mask_1;
    std::shared_ptr<cv::Mat> element_mask_2;
    std::shared_ptr<cv::Mat> element_depth_1;
    std::shared_ptr<cv::Mat> element_depth_2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr element_pcl_1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr element_pcl_2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_merged;
    std::vector<json> parts_description;
  };

  struct Item
  {
    int32_t id = -1;
    uint32_t item_cam1_id = 0;
    uint32_t item_cam2_id = 0;
    std::string item_id_hash{};
    std::string label{};
    Eigen::Affine3f pose;
    std::vector<ItemElement> item_elements;
    bool isEstimationValid = true;
  };
} // namespace estimate_shape

#endif
