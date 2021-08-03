#ifndef FIT_OCTOPHERE_HPP
#define FIT_OCTOPHERE_HPP

// ___PCL___
#include <pcl/common/distances.h>
#include <pcl/segmentation/sac_segmentation.h>

// ___Package___
#include "ifit_method/ifit_method.hpp"

namespace estimate_shape
{
  class FitOctoSphere : public IFitMethod
  {
  public:
    FitOctoSphere(estimate_shape::CameraParameters cam1_params, estimate_shape::CameraParameters cam2_params, std::vector<std::string> &args, json &default_parameters, bool debug = false);
    virtual ~FitOctoSphere();
    virtual int fit(std::vector<ItemElement> &item_elements) override;
    virtual int setLabelData(std::vector<Label> &labels) override;

  private:
    int _sortParts(std::vector<ItemElement> &item_elements);
    int _fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, uint8_t model_type, pcl::ModelCoefficients &out_model_coefficients);
    int _fitBothParts(std::vector<ItemElement> &item_elements);
    int _assignHeadData(const Eigen::Vector3f &pos, const Eigen::Quaternionf &rot, json &pose);
    int _assignOctomap(ItemElement &item_element);
    int _handleUnsupportedElements(std::vector<ItemElement> &item_elements);
    std::vector<std::string> _labels;
    std::map<std::string, std::string> _elements_fit_methods;
  };
} // namespace estimate_shape

#endif
