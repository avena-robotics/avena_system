#ifndef CREATE_PTCLD_HPP
#define CREATE_PTCLD_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
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

#include "helpers_vision/helpers_vision.hpp"
#include "structures.hpp"

#define log_debug std::cout << __func__ << " " << __LINE__ << std::endl;


namespace create_ptcld
{


    using transform_map = std::map<std::string, Eigen::Affine3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>>;

    struct ROI
    {
        int x;
        int y;
        int dx;
        int dy;
    };

    int cutDepthMapWithMask(cv::Mat &mask, cv::Mat &depth, cv::Mat &out_item_depth, bool reversed = false);
    void computeMaskForIndex(camera_data_ptr &cams_data, size_t idx_of_output_mask, element &element);
    bool compareMasks(cv::Mat &mask1, cv::Mat &mask2, float tresh);
    int voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, float leaf_size);
    int transformCloudToFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, camera_data_ptr &cams_data, size_t &cam_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
    int computeBB(cv::Mat &mask, ROI &out_roi);
    int reconstructPointCloud(WorkspaceArea &workspace_area, cv::Mat &depth_image, cv::Mat &rgb, camera_data_ptr cams_params, ROI roi, size_t cam_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out);
    int convertPtcldToMask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, camera_data_ptr &cams_data, size_t &mask_cam_idx, cv::Mat &out_mat);
    int statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud);
    int createPtcld(WorkspaceArea &workspace_area, element &el, cv::Mat &depth, cv::Mat &rgb, size_t camera_index, camera_data_ptr &cams_params, bool remove_shadows);
    void bilateralFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_out_cloud);
    void normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &out_normals);
    void filter_shadows(float treshold, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &input_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud, pcl::PointIndices::Ptr &pi);

} // namespace robot

#endif
