#ifndef FILTER_DETECTIONS_H
#define FILTER_DETECTIONS_H

// c++
#include <vector>
#include <stdexcept>
#include <functional>
#include <memory>
#include <thread>
#include <iostream>
#include <string>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <nlohmann/json.hpp>
#include "helpers_vision/helpers_vision.hpp"
#include "helpers_commons/helpers_commons.hpp"

//ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "custom_interfaces/action/simple_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "custom_interfaces/msg/detections.hpp"
#include "custom_interfaces/srv/data_store_detectron_insert.hpp"

using json = nlohmann::json;

struct LabeledMasks
{
  std::string label = "";
  std::vector<cv::Mat> masks;
};

class FilterDetection : public rclcpp::Node, public helpers::WatchdogInterface
{
public:
  using FilterDetectionAction = custom_interfaces::action::SimpleAction;
  using GoalHandleFilterDetection = rclcpp_action::ServerGoalHandle<FilterDetectionAction>;

  void initNode() override;
  void shutDownNode() override;

  FilterDetection(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), bool debug = false);
  ~FilterDetection();

private:
  helpers::Watchdog::SharedPtr _watchdog;

  int _readLabels();
  int _assignInputData(custom_interfaces::msg::Detections::SharedPtr &detect_msg);
  int _assignFilteredData(custom_interfaces::msg::Detections::UniquePtr &detect_msg);
  bool _checkOverlay(cv::Mat &mask1, cv::Mat &mask2, float overlay_margin);
  bool _checkOverlay(cv::Mat &item_mask, cv::Mat &element_mask);

  int _handleMultipleDetections();
  int _reconstructMissingMasks();
  int _findUncompleteItems(std::vector<LabeledMasks> &out_wrappers, std::vector<LabeledMasks> &out_components);
  int _genereteMissingComponents(cv::Mat &wrapper, std::string wrapper_label, std::vector<LabeledMasks> &out_matched_elements);

  void prepareDetectionVectors(custom_interfaces::msg::Detections::SharedPtr &detect_msg);
  std::vector<LabeledMasks> _detections_cam1;
  std::vector<LabeledMasks> _detections_cam2;

  std::map<std::string, bool> _items;
  std::map<std::string, bool> _elements;
  std::map<std::string, std::set<std::string>> _components;

  //ROS
  rclcpp_action::Server<FilterDetectionAction>::SharedPtr _action_server;
  rclcpp::Publisher<custom_interfaces::msg::Detections>::SharedPtr _publisher;
  rclcpp::Subscription<custom_interfaces::msg::Detections>::SharedPtr _detections_sub;
  custom_interfaces::msg::Detections::SharedPtr _detections_msg;
  rclcpp::Client<custom_interfaces::srv::DataStoreDetectronInsert>::SharedPtr _detectron_insert;

  void _execute(const std::shared_ptr<GoalHandleFilterDetection> goal_handle);
  rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FilterDetectionAction::Goal> goal);
  rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleFilterDetection> goal_handle);
  void _handleAccepted(const std::shared_ptr<GoalHandleFilterDetection> goal_handle);

  bool _debug;
  builtin_interfaces::msg::Time _last_processed_msg_timestamp;
};

#endif //FILTER_DETECTIONS_H
