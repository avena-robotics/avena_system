#ifndef ESTIMATE_SHAPE_FIT_BOWL_HPP
#define ESTIMATE_SHAPE_FIT_BOWL_HPP

// ___PCL___
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Geometry>
#include <pcl/common/distances.h>

// #include <pcl/keypoints/harris_3d.h>
// #include <pcl/keypoints/keypoint.h>

// ___Package___
#include "estimate_shape_ifit_method/ifit_method.hpp"
// #include "visualization.hpp"

namespace estimate_shape
{
  class FitBowl : public IFitMethod
  {
  public:
    FitBowl(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitBowl();
    virtual int fit(Item &item) override;
    virtual int checkEstimation(Item &item) override;
    virtual int setLabelData(const std::vector<Label> &labels) override;


  private:
    void _clear();
    int _computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Matrix3f &rotational_matrix_OBB, pcl::PointXYZ &position_OBB);
    // int _calculateSphereParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, pcl::ModelCoefficients &coeffients, float radius_min = 0.086, float radius_max = 0.088);
    int _computeSphereRadius(float big_radius, float smaller_radius, float height, float & sphere_radius);
    int _calculateSphereParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, pcl::ModelCoefficients &coeffients, float radius_min , float radius_max );
    int _computeInitialPose(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, pcl::ModelCoefficients &coeffients, float radius_min, Eigen::Affine3f &out_pose);
    int _refineRotation(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, Eigen::Affine3f &transform, float radius);
    int _assignData(Item &item, pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, Eigen::Affine3f &pose);
    int _checkData(ItemElement &item_element);

    bool _show;

    float _height;
    float _big_radius;
    float _small_radius;
    json _raw_data;
  };
} // namespace estimate_shape

#endif
