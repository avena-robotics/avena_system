#ifndef PTCLD_MERGER_HPP
#define PTCLD_MERGER_HPP

// ___CPP___
#include <memory>
#include <functional>

// ___PCL___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

// ___ROS2___
#include <rclcpp/rclcpp.hpp>

// ___Avena___
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

namespace ptcld_merger
{
  class PtcldMerger : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    PtcldMerger(const rclcpp::NodeOptions &options);
    ~PtcldMerger() = default;
    virtual void initNode() override;
    virtual void shutDownNode() override;

  private:
    helpers::Watchdog::SharedPtr _watchdog;
    std::unordered_map<std::string, Eigen::Affine3f> _cameras_transforms;
  };

} // namespace ptcld_merger

#endif
