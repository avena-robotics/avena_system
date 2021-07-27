#ifndef CREATE_PTCLD_HPP
#define CREATE_PTCLD_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl-1.10/pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "structures.hpp"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl-1.10/pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/conversions.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/shadowpoints.h>

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <cstdlib>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <limits>
// ___ROS2___
#include "rclcpp/rclcpp.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

namespace robot
{

    using transform_map = std::map<std::string, Eigen::Affine3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>>;

    struct ROI
    {
        int x;
        int y;
        int dx;
        int dy;
    };

    class CreatePtcld
    {
    public:
        CreatePtcld();
        int setCameraParams(transform_map &transform, std::map<std::string, CameraParameters> &cam_params);

        int cutDepthMapWithMask(cv::Mat &mask, cv::Mat &depth, cv::Mat &out_item_depth, bool reversed = false);

        int passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string axis, float min_limit, float max_limit);
        int voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, float leaf_size);
        int clusterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointIndices> &out_cluster_indices, float cluster_tolerance = 0.01, int min_cluster_size = 20, int max_cluster_size = 200000);
        int transformCloudToFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
        int computeBB(cv::Mat &mask, ROI &out_roi);
        int reconstructPointCloud(WorkspaceArea workspace_area, cv::Mat &depth_image, cv::Mat &rgb, CameraParameters cam_params, ROI roi, std::string frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shadow);
        bool removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcld, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_ptcld);
        int radiusOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcld, float radius, size_t neighbors);

        int convertPtcldToMask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat &out_mat, float res_scale = 1.0);
        int compareItemsBetweenCameras(cv::Mat &reconstructed_mask, std::string label, std::vector<item_cam_t> &detections_cam2, uint32_t &out_corresponding_item_id);
        void getCamerasParameters(const rclcpp::Node::SharedPtr &node);
        int statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud);

        int findCorespondingItem(item_cam_t &item, std::vector<item_cam_t> &detections_cam2, uint32_t &out_corresponding_item_id);

        CameraParameters obtainCameraParameters();
        int createPtcld(WorkspaceArea workspace_area, cv::Mat &mask, cv::Mat &depth, cv::Mat &rgb, int camera_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shadow);

        Frames _frames;
        std::map<std::string, CameraParameters> _camera_parameters;
        transform_map _camera_transform;
    };
} // namespace robot

#endif
