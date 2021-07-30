#ifndef ESTIMATE_SHAPE_COMPONENT_HPP_
#define ESTIMATE_SHAPE_COMPONENT_HPP_

// ___CPP___
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <optional>

// __ROS__
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// __AVENA__
// #include "helpers.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"
#include "custom_interfaces/msg/items.hpp"
#include "custom_interfaces/msg/item.hpp"
#include "custom_interfaces/msg/item_element.hpp"
#include "custom_interfaces/action/simple_action.hpp"
#include "estimate_shape_manager/estimate_shape_manager.hpp"
#include "estimate_shape/visibility_control.h"
#include "estimate_shape_commons/commons.hpp"

#include "custom_interfaces/srv/data_store_items_insert.hpp"
#include "custom_interfaces/srv/data_store_items_select.hpp"

using namespace std::chrono_literals;


namespace estimate_shape
{
  using ItemsMsg = custom_interfaces::msg::Items;
  using ItemMsg = custom_interfaces::msg::Item;
  using ItemElementMsg = custom_interfaces::msg::ItemElement;

  using Response = custom_interfaces::srv::DataStoreItemsSelect::Response;

  class EstimateShape : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    using SimpleAction = custom_interfaces::action::SimpleAction;
    using GoalHandleSimpleAction = rclcpp_action::ServerGoalHandle<SimpleAction>;

    ESTIMATE_SHAPE_PUBLIC
    explicit EstimateShape(const rclcpp::NodeOptions &options);

    void initNode() override;
    void shutDownNode() override;

    ESTIMATE_SHAPE_PUBLIC
    ~EstimateShape();

  private:
    helpers::Watchdog::SharedPtr _watchdog;
    int _getParametersFromServer(std::vector<Label> &out_labels_parameters, std::vector<estimate_shape::CameraParameters> &out_cameras_parameters);
    Response::SharedPtr  _estimateShapeProcessing(const ItemsMsg::SharedPtr &input_items, std::string item_label, std::string fit_method);
    int _convertTopicDataToStructures(const ItemsMsg::SharedPtr &input_items, std::vector<Item> &out_items);
    int _convertStructuresToTopicData(const std::vector<Item> &item_results, Response::SharedPtr &out_estimate_shape);
    Response::UniquePtr _prepareOutputData(const ItemsMsg::SharedPtr &input_items);
    int _validateInput(const ItemsMsg::SharedPtr &input_items);



    void _getData();

    // Action server methods
    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const SimpleAction::Goal> goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleSimpleAction> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandleSimpleAction> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleSimpleAction> goal_handle);

    rclcpp_action::Server<SimpleAction>::SharedPtr _estimate_shape_server;

    // ROS publisher, subscribers, action server
    rclcpp::Subscription<custom_interfaces::msg::Items>::SharedPtr _compose_items_sub;

    rclcpp::Publisher<Response>::SharedPtr _estimate_shape_pub;
    custom_interfaces::msg::Items::SharedPtr _compose_items_msg;

    EstimateShapeManager::UniquePtr _estimate_shape_manager;
    std::vector<estimate_shape::CameraParameters> _cams_params;
    std::vector<Label> _labels;
    builtin_interfaces::msg::Time _last_processed_msg_timestamp;



    rclcpp::Client<custom_interfaces::srv::DataStoreItemsSelect>::SharedPtr _items_select_client;
    rclcpp::Client<custom_interfaces::srv::DataStoreItemsInsert>::SharedPtr _items_insert_client;




  };

} // namespace estimate_shape

#endif // ESTIMATE_SHAPE_COMPONENT_HPP_
