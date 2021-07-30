#ifndef ESTIMATE_SHAPE_FITCYLINDER_HPP
#define ESTIMATE_SHAPE_FITCYLINDER_HPP

// ___PCL___
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/distances.h>

// #include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/prosac.h>

#include <Eigen/Core>
// ___Package___
#include "estimate_shape_ifit_method/ifit_method.hpp"

namespace estimate_shape
{
  class FitCylinder : public IFitMethod
  {
  public:
    FitCylinder(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitCylinder();
    virtual int fit(Item &item) override;

  private:
    float _radius_min;
    float _radius_max;
    int _estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, std::string frame, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud_with_normals);
    void _fitModelNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr obj_ptcld, uint8_t model_type, pcl::ModelCoefficients &out_model_coefficients);
    float _computeCylinderHeight(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld);
    pcl::PointXYZ _computeCylinderCentroid(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, std::vector<float> &out_obj_dim);
    int _computeCylinderRotation(pcl::ModelCoefficients::Ptr cylinder_coefficients, Eigen::Affine3f &out_obj_pose);
    // Eigen::Vector3f _findAxisVector(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr obj_ptcld);
    int _computeCylinderRadius(pcl::PointXYZ &center, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Affine3f &out_obj_pose, float &out_radius);
  };
} // namespace estimate_shape

#endif
