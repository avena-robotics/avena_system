#ifndef ESTIMATE_SHAPE_FIT_SPHERE_HPP
#define ESTIMATE_SHAPE_FIT_SPHERE_HPP

// ___PCL___
#include <pcl/segmentation/sac_segmentation.h>

// ___Package___
#include "estimate_shape_ifit_method/ifit_method.hpp"

namespace estimate_shape
{
  class FitSphere : public IFitMethod
  {
  public:
    FitSphere(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitSphere();
    virtual int fit(Item &item) override;

  private:
    float _radius_min;
    float _radius_max;
    int _fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, uint8_t model_type, pcl::ModelCoefficients &out_model_coefficients);
  };
} // namespace estimate_shape

#endif
