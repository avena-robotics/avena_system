#ifndef RGBD_SYNC_COMPONENT_HPP
#define RGBD_SYNC_COMPONENT_HPP

// ___CPP___
#include <functional>
#include <memory>
#include <string>
#include <tuple>

// __ROS__
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter.hpp>
// #include <rclcpp/executors.hpp>
// #include <rclcpp/executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
// ___AVENA___
#include "rgbd_sync/visibility_control.h"
#include "custom_interfaces/action/simple_action.hpp"
#include "custom_interfaces/msg/rgb_images.hpp"
#include "custom_interfaces/msg/depth_images.hpp"
#include "custom_interfaces/msg/ptclds.hpp"
#include "custom_interfaces/msg/rgbd_sync.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

#include "custom_interfaces/srv/data_store_rgbd_sync_insert.hpp"
#include "custom_interfaces/srv/data_store_rgbd_sync_select.hpp"

using namespace std::chrono_literals;

namespace rgbd_sync
{
  using namespace custom_interfaces::msg; // usage only in this namespace, so not a big problem
  using Request = custom_interfaces::srv::DataStoreRgbdSyncInsert::Request;

  class RgbdSynchronizer : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    explicit RgbdSynchronizer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    using Action = custom_interfaces::action::SimpleAction;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
    
    void initNode() override;
    void shutDownNode() override;

  private:
    helpers::Watchdog::SharedPtr _watchdog;

    Request::SharedPtr _prepareOutputMessages(const RgbImages::SharedPtr &rgb_images, const DepthImages::SharedPtr &depth_images, const Ptclds::SharedPtr &ptclds );

    //ROS
    rclcpp_action::Server<Action>::SharedPtr _action_server;
    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, Action::Goal::ConstSharedPtr goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandle> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandle> goal_handle);
    void _execute(const std::shared_ptr<GoalHandle> goal_handle);
    void _initializeServers();



    rclcpp::Client<custom_interfaces::srv::DataStoreRgbdSyncInsert>::SharedPtr _client;
    rclcpp::Subscription<custom_interfaces::msg::RgbImages>::SharedPtr _rgb_images_sub;
    rclcpp::Subscription<custom_interfaces::msg::DepthImages>::SharedPtr _depth_images_sub;
    rclcpp::Subscription<custom_interfaces::msg::Ptclds >::SharedPtr _ptclds_sub;

    RgbImages::SharedPtr _rgb_images_data;
    DepthImages::SharedPtr _depth_images_data;
    Ptclds::SharedPtr _ptclds_data;


    // rclcpp::Publisher<Response>::SharedPtr _rgbd_sync_publisher;
    bool _cant_touch_this;

  };

} // namespace

#endif
