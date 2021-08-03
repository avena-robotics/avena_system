#ifndef ESTIMATE_SHAPE_FIT_PLATE_HPP
#define ESTIMATE_SHAPE_FIT_PLATE_HPP

// ___PCL___
#include <Eigen/Geometry>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

// ___Package___
#include "estimate_shape_ifit_method/ifit_method.hpp"
// #include "visualization.hpp"

namespace estimate_shape
{
  class FitPlate : public IFitMethod
  {
  public:
    FitPlate(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitPlate();
    virtual int fit(Item &item) override;
    virtual int checkEstimation(Item &item) override;
    virtual int setLabelData(const std::vector<Label> &labels) override;

  private:
    void _clear();
    int _computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Matrix4f &obb_transform);
    int _filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed, Eigen::Vector4f &centroid, float threshold = 3);
    int _calculateSphereParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed, pcl::ModelCoefficients &coeffients, float radius_min = 0.1, float radius_max = 0.3);
    int _computeSphereRadius(float big_radius, float smaller_radius, float height, float &sphere_radius);
    int _computeAndSavePlateData(Item &item, pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed, pcl::ModelCoefficients coeffients);

    pcl::PointCloud<pcl::PointXYZ>::Ptr _item_ptcld_transformed;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _item_ptcld;
    Eigen::Matrix4f _obb_transform;
    bool _show;
    const float _plate_bigger_radius = 0.123;
    const float _plate_smaller_radius = 0.082;
    const float _plate_height = 0.0571;
    json _raw_data;
  };
} // namespace estimate_shape

#endif
