#ifndef HELPERS_VISION__COMMONS_HPP_
#define HELPERS_VISION__COMMONS_HPP_

// ___CPP___
#include <memory>
#include <chrono>
#include <optional>
#include <random>

// ___OpenCV___
#include <opencv2/opencv.hpp>

// ___PCL___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include <set>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/approximate_voxel_grid.h>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/transform_listener.h>

// ___Eigen___
#include <eigen3/Eigen/Eigen>

// ___JSON___
#include <nlohmann/json.hpp>

// ___Helpers commons___
#include "helpers_commons/structures.hpp"

namespace helpers
{
  using json = nlohmann::json;
  namespace vision
  {
    Eigen::Matrix3f assignRotationMatrixAroundZ(float angle);
    Eigen::Matrix3f assignRotationMatrixAroundY(float angle);
    Eigen::Matrix3f assignRotationMatrixAroundX(float angle);
    bool checkPointsAmmount(pcl::PointCloud<pcl::PointXYZ>::Ptr &obj_ptcld, size_t k_neighbours);
    int extract(pcl::PointIndices::Ptr indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud);
    int extract(pcl::PointIndices::Ptr indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_out_cloud);
    int extract(std::vector<int> &indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_out_cloud);
    int extract(std::vector<int> &indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud);
    int rotatePtcldAroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &obj_ptcld, Eigen::Quaternionf rot, pcl::PointXYZ rot_center, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_ptcld);
    int flattenPlaneCloud(Eigen::Quaternionf &z_rot, pcl::PointXYZ &centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_plane_cloud);
    int hull2D(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_hull);
    Eigen::Matrix3f findRotationFromHullPoints(pcl::PointXYZ &hullPoint, pcl::PointXYZ &followingPoint);
    int rotateHull(Eigen::Matrix3f &rotation, pcl::PointCloud<pcl::PointXYZ>::Ptr hull, pcl::PointCloud<pcl::PointXYZ>::Ptr out_hull);
    int statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud, int mean_k = 100, float std_dev_mul_thresh = 1.96);
    int passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string axis, float min_limit, float max_limit, bool negative = false);
    json assignJsonFromPosition(Eigen::Vector3f &position);
    json assignJsonFromQuaternion(Eigen::Quaternionf &orientation);
    Eigen::Vector3f assignPositionFromJson(json &position);
    Eigen::Quaternionf assignQuaternionFromJson(json &orientation);
    int voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size);
    int approximateVoxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size);
    bool radiusOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld, float radius, size_t neighbors);
    Eigen::Quaternionf computeQuaternionFromAngle(float angle, Eigen::Vector3f reference);
    Eigen::Quaternionf rotateAroundAxis(std::string axis_name, float angle);

    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr makeSharedPcl()
    {
      return pcl::make_shared<pcl::PointCloud<PointT>>();
    }

    /**
     * @brief Get the Transform Stamped object
     * 
     * @param target_frame 
     * @param source_frame 
     * @param timeout 
     * @return std::optional<geometry_msgs::msg::TransformStamped> 
     */
    std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout = std::chrono::seconds(1));

    /**
     * @brief Get the Camera Transform Stamped object
     * 
     * @param target_frame 
     * @param source_frame 
     * @param timeout 
     * @return std::optional<geometry_msgs::msg::TransformStamped> 
     */
    std::optional<geometry_msgs::msg::TransformStamped> getCameraTransformStamped(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout = std::chrono::seconds(1));

    /**
     * @brief Get the Transform Affine object
     * 
     * @param target_frame 
     * @param source_frame 
     * @param timeout 
     * @return std::optional<Eigen::Affine3f> 
     */
    std::optional<Eigen::Affine3f> getTransformAffine(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout = std::chrono::seconds(1));

    /**
     * @brief Get the Camera Transform Affine object
     * 
     * @param target_frame 
     * @param source_frame 
     * @param timeout 
     * @return std::optional<Eigen::Affine3f> 
     */
    std::optional<Eigen::Affine3f> getCameraTransformAffine(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout = std::chrono::seconds(1));

    /**
     * @brief Get the Camera Intrinsic object
     * 
     * @param node_topics_interface 
     * @param camera_frame 
     * @param timeout 
     * @return std::optional<CameraIntrinsic> 
     */
    std::optional<CameraIntrinsic> getCameraIntrinsic(rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface, const std::string &camera_frame, const std::chrono::duration<float> &timeout = std::chrono::seconds(1));
  } // namespace vision
} // namespace helpers

#endif // HELPERS_VISION__COMMONS_HPP_
