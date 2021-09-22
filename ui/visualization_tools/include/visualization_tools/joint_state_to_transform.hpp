#ifndef VISUALIZATION_TOOLS__JOINT_STATE_TO_TF_HPP_
#define VISUALIZATION_TOOLS__JOINT_STATE_TO_TF_HPP_

// ___CPP___
// #include <chrono>
// #include <map>
// #include <memory>
// #include <string>
// #include <vector>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

// ___Package___
#include "visualization_tools/visibility_control.h"

namespace visualization_tools
{
  using MimicMap = std::map<std::string, urdf::JointMimicSharedPtr>;

  class SegmentPair final
  {
  public:
    explicit SegmentPair(
        const KDL::Segment &p_segment,
        const std::string &p_root,
        const std::string &p_tip)
        : segment(p_segment), root(p_root), tip(p_tip) {}

    KDL::Segment segment;
    std::string root;
    std::string tip;
  };

  class JointStateToTransform
  {
  public:
    JointStateToTransform() = default;
    // explicit JointStateToTransform(const std::string &urdf_xml);
    ~JointStateToTransform() = default;
    void initModel(const std::string &urdf_xml);

    std::vector<geometry_msgs::msg::TransformStamped> getMovingTransforms(
        std::map<std::string, double> &joint_positions,
        const builtin_interfaces::msg::Time &time = builtin_interfaces::msg::Time());
    const std::vector<geometry_msgs::msg::TransformStamped> &getFixedTransforms() const
    {
      return _fixed_transforms;
    }

  private:
    std::vector<geometry_msgs::msg::TransformStamped> _getFixedTransformsImpl();
    void _addChildren(const KDL::SegmentMap::const_iterator segment);
    void _setupURDF(const std::string &urdf_xml);

    std::map<std::string, SegmentPair> _segments;
    std::map<std::string, SegmentPair> _segments_fixed;
    std::unique_ptr<urdf::Model> _model;
    MimicMap _mimic;
    std::vector<geometry_msgs::msg::TransformStamped> _fixed_transforms;

    const rclcpp::Logger JOINT_STATE_TO_TF_LOGGER = rclcpp::get_logger("joint_state_to_tf");
  };

} // namespace visualization_tools

#endif // VISUALIZATION_TOOLS__JOINT_STATE_TO_TF_HPP_
