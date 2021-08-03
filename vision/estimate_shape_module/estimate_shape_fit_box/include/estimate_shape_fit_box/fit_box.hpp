#ifndef ESTIMATE_SHAPE_FIT_BOX_HPP
#define ESTIMATE_SHAPE_FIT_BOX_HPP

// ___PCL___
#include <pcl/segmentation/sac_segmentation.h>

// ___Package___
#include "estimate_shape_ifit_method/ifit_method.hpp"

namespace estimate_shape
{
  class FitBox : public IFitMethod
  {
  public:
    FitBox(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitBox();
    virtual int fit(Item &item) override;

  private:
    struct Dimensions
    {
      Dimensions(float min = 0.0f, float max = 0.0f)
          : min(min), max(max)
      {
        size = max - min;
        center_displacement = (max + min) / 2;
      };
      float min, max, size, center_displacement;
      bool operator<(const Dimensions &d) const
      {
        return size < d.size;
      };
    };

    int _fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices &out_model_inlierIndices, pcl::ModelCoefficients &out_model_coefficients);
    int _fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients &out_model_coefficients);
    int _getBoxData(std::vector<ItemElement> &item_elements, pcl::PointCloud<pcl::PointXYZ>::Ptr box_ptcld);
    int _findInitialAffine(pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud, Eigen::Affine3f &out_initial_affine);
    int _alignBoxAxes(pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud, Eigen::Affine3f &box_affine, std::vector<Dimensions> &out_dim);
    int _assignBoxData(std::vector<Dimensions> &dim, const Eigen::Affine3f &box_affine, Item &item);
  };
} // namespace estimate_shape

#endif
