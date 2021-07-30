#ifndef FIT_TOOL_HPP
#define FIT_TOOL_HPP

// ___Package___
#include "estimate_shape_fit_box/fit_box.hpp"
#include "estimate_shape_ifit_method/ifit_method.hpp"
#include "pcl/point_types_conversion.h"

namespace estimate_shape
{
  class FitTool : public IFitMethod
  {
  public:
    FitTool(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitTool();

    /**
     * @brief Tool fitting method
     * 
     * @param item CAUTION! elements need to be sorted before use of this function! so that handle is 1st element in this vector!
     * @return position and orientation of handle assigned to 1st input element.
     */
    virtual int fit(Item &item) override;

  private:
    int _fitBox(Item &item);
    // int _sortParts(std::vector<ItemElement> &item_elements);
    int _fixRotation(Item &item);
    int _extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_plane_cloud);
    int _alignAxisWithColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &handle_merged, Eigen::Quaternionf &orient);
    int _getDirectionVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &handle_merged, Eigen::Vector3f &direction_vec, float &hue);
    int _fixBoxEstimation(Item &item);
    int _getHue(pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud, float &hue);

    // std::vector<std::string> _labels;
    std::map<std::string, std::string> _elements_fit_methods;
    const int HANDLE = 0;



    static constexpr int RED = 0;
    static constexpr int LIME = 120;
    static constexpr int BLUE = 240;
    static constexpr int YELLOW = 60;
    static constexpr int MAGENTA = 300;
    static constexpr int CYAN = 180;


  };
} // namespace estimate_shape


int _hue_treshold;
#endif
