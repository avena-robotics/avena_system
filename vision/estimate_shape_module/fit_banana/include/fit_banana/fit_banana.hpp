#ifndef FIT_BANANA_HPP
#define FIT_BANANA_HPP

// ___PCL___
#include <pcl/common/common.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>

// ___Package___
#include "visualization.hpp"
#include "ifit_method/ifit_method.hpp"

#define MAX_NO_OF_SEGMENTS 6

namespace estimate_shape
{
  class FitBanana : public IFitMethod
  {
  public:
    FitBanana(estimate_shape::CameraParameters cam1_params, estimate_shape::CameraParameters cam2_params, std::vector<std::string> &args, json &default_parameters, bool debug = false);
    virtual ~FitBanana();
    virtual int fit(std::vector<ItemElement> &item_elements) override;

  private:
    bool _readMergedPtcld(std::string table_name, pcl::PointCloud<pcl::PointXYZ>::Ptr out_db_ptcld, std::vector<uint32_t> &out_scene_ids);
    bool _filterPtcld(pcl::PointCloud<pcl::PointXYZ>::Ptr out_db_ptcld);
    bool _computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Matrix4f &obb_transform, Eigen::Quaternionf &obb_rot, Eigen::Vector3f &obb_pos);
    bool _dividePtcld(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_transformed_ptcld, std::vector<pcl::Indices> &out_ptcld_segments_indices);
    bool _createCylinders(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_transformed_ptcld, float segment_len, Eigen::Matrix4f &obb_transform, std::vector<pcl::Indices> &ptcld_segments_indices, std::vector<pcl::ModelCoefficients> &out_cs_coefficients);
    bool _extractPtcld(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_ptcld, pcl::Indices &ptcld_segment_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr segment_ptcld);
    int _passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, float min_limit, float max_limit, bool negative, pcl::Indices &out_cloud_indices);
    bool _fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_ptcld, float segment_len, pcl::ModelCoefficients::Ptr out_c_coefficients, Eigen::Affine3f &c_pose, std::vector<float> &c_dim);
    int _computeCylinderPoseAndSize(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Affine3f &cam_pose, pcl::ModelCoefficients::Ptr cylinder_coefficients, Eigen::Affine3f &out_obj_pose, std::vector<float> &out_obj_dim);
    float _computeCylinderHeight(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld);
    float _computeCylinderHeight_fix(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld);
    pcl::PointXYZ _computeCylinderCentroid(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, std::vector<float> &out_obj_dim);
    int _computeCylinderRotation(pcl::ModelCoefficients::Ptr cylinder_coefficients, Eigen::Affine3f &out_obj_pose);
    int _rotatePtcldAroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Quaternionf rot, pcl::PointXYZ rot_center, pcl::PointCloud<pcl::PointXYZ>::Ptr out_ptcld);
    bool _assignBananaElements(std::vector<ItemElement> &item_elements);

    pcl::PointCloud<pcl::PointXYZ>::Ptr _item_ptcld;
    std::vector<pcl::ModelCoefficients> _cylinders_coefficients;
    std::vector<uint32_t> _scene_ids;
    Eigen::Matrix4f _obb_transform;
    Eigen::Quaternionf _obb_rot;
    Eigen::Vector3f _obb_pos;
    std::vector<pcl::Indices> _item_ptcld_segments_indices;
    Eigen::Affine3f _virtual_cam_pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _item_ptcld_transformed;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _segments_ptclds;
    std::vector<Eigen::Vector3f> _segments_positions;
    std::vector<Eigen::Quaternionf> _segments_orientations;
    std::vector<std::vector<float>> _segments_dims;
    size_t _segments_no;
    std::vector<std::string> _segments_fit_method;
    const std::string _element_label_name = "segment";
    const std::string _element_fit_method = "cylinder";
    float _segment_len;
    std::vector<std::stringstream> _serialized_items_ptclds;
  };
} // namespace estimate_shape

#endif
