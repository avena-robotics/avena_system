#ifndef FIT_KNIFE_HPP
#define FIT_KNIFE_HPP

// ___Package___
#include "estimate_shape_fit_tool/fit_tool.hpp"
#include "estimate_shape_ifit_method/ifit_method.hpp"

namespace estimate_shape
{
  class FitKnife : public IFitMethod
  {
  public:
    FitKnife(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitKnife();
    virtual int fit(Item &item) override;
    int setLabelData(const std::vector<Label> &labels) override;

  private:
    int _fitTool(Item &item);
    int _sortParts(Item &item);
    int _getHandleAsItem(Item &item, Item &handle);

    std::vector<std::string> _labels;
    const int HANDLE = 0;
    const int BLADE = 1;
    float knife_center_to_handle_distance;

  };
} // namespace estimate_shape

#endif
