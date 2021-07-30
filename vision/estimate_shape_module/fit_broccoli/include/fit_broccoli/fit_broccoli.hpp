#ifndef FIT_BROCCOLI_HPP
#define FIT_BROCCOLI_HPP

// ___PCL___
#include <pcl/common/distances.h>
#include <pcl/features/moment_of_inertia_estimation.h>

// ___Package___
#include "ifit_method/ifit_method.hpp"

namespace estimate_shape
{
  class FitBroccoli : public IFitMethod
  {
  public:
    FitBroccoli(CameraParameters cam1_params, CameraParameters cam2_params, std::vector<std::string> &args, json &default_parameters, bool debug = false);
    virtual ~FitBroccoli();
    virtual int fit(std::vector<ItemElement> &item_elements) override;

  private:
    float _radius_min;
    float _radius_max;
    float _voxel_size;
    int _sortBroccoliParts(std::vector<ItemElement> &item_elements);
    int _computeLegPoseAndSize(pcl::PointCloud<pcl::PointXYZ>::Ptr head, pcl::PointCloud<pcl::PointXYZ>::Ptr leg, Eigen::Affine3f &out_leg_pose, std::vector<float> &out_leg_dim);
    int _computeHeadPoseAndSize(pcl::PointCloud<pcl::PointXYZ>::Ptr head, Eigen::Affine3f &out_head_pose, std::vector<float> &out_head_dim);
    int _assignLegData(Eigen::Affine3f &leg_pose, std::vector<float> &leg_dim, ItemElement &leg_element);
    int _assignHeadData(Eigen::Affine3f &head_pose, std::vector<float> &head_dim, ItemElement &head_element);
    int _filterInputPointclouds(std::vector<ItemElement> &item_elements, pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_head, pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_leg);
  };
} // namespace estimate_shape

#endif
