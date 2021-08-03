#ifndef ESTIMATE_SHAPE_FIT_CUTTING_BOARD_HPP
#define ESTIMATE_SHAPE_FIT_CUTTING_BOARD_HPP

// ___PCL___
#include "estimate_shape_ifit_method/ifit_method.hpp"
#include "estimate_shape_fit_box/fit_box.hpp"

namespace estimate_shape
{
  class FitCuttingBoard : public IFitMethod
  {
  public:
    FitCuttingBoard(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters);
    virtual ~FitCuttingBoard();
    virtual int fit(Item &item) override;

  private:
    void _clear();
    bool _removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld);
    int _fixCuttingBoardOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld, Item &item);
    int _reinforceBoardOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr item_cloud, Item &item, float cut_distance);
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> _cuttingBoardSplit(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld);
    // Eigen::Affine3f _readAffineFromItem(const Item &item);
    int _assignData(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_map, Item &item);

    bool _calculatePlanePosition(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld, pcl::ModelCoefficients::Ptr coefficients);
    int _removePointsAboveBoard(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld, bool sac_show);
    int _fitBox(Item &item);

    bool _show;
    //Order matter here! change just names - not order
    std::vector<std::string> _parts = {"handle", "board"};
  };
} // namespace estimate_shape

#endif