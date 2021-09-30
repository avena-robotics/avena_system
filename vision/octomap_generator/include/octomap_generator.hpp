#ifndef OCTOMAP_GENERATOR_H
#define OCTOMAP_GENERATOR_H

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
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>


#include <nlohmann/json.hpp>
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

//ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "custom_interfaces/action/simple_action.hpp"
#include "custom_interfaces/msg/rgbd_sync.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "custom_interfaces/srv/data_store_items_select.hpp"
#include "custom_interfaces/srv/data_store_scene_insert.hpp"
#include "ptcld_transformer/ptcld_transformer.hpp"
#include "robot_self_filter/robot_self_filter.hpp"

using namespace std::chrono_literals;

  const float ROBOT_SELF_FILTER_DEFAULT_OFFSET = 0.05;

namespace octomap_generator
{

  using json = nlohmann::json;

  typedef std::chrono::system_clock::time_point TimeVar;
  #define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
  #define timeNow() std::chrono::high_resolution_clock::now()

  // using namespace ;

  class OctomapGenerator : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    using OctomapGeneratorAction = custom_interfaces::action::SimpleAction;
    using GoalHandleOctomapGenerator = rclcpp_action::ServerGoalHandle<OctomapGeneratorAction>;


    OctomapGenerator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~OctomapGenerator();

    virtual void initNode() override;
    virtual void shutDownNode() override;


  private:
    helpers::Watchdog::SharedPtr _watchdog;
    rclcpp::Client<custom_interfaces::srv::DataStoreRgbdSyncSelect>::SharedPtr _rgbd_sync_select_client;
    rclcpp::Client<custom_interfaces::srv::DataStoreSceneInsert>::SharedPtr _scene_insert_client;

    void _getData(custom_interfaces::msg::RgbdSync::SharedPtr &cameras_data);

    int _validateInputs(custom_interfaces::msg::RgbdSync::SharedPtr cameras_data);

    std::shared_ptr<ptcld_transformer::PtcldTransformer> _pointCloud_transformer;
    std::shared_ptr<robot_self_filter::RobotSelfFilter> _robot_self_filter;

    rclcpp_action::Server<OctomapGeneratorAction>::SharedPtr _action_server;

    int _sendDataToDB(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &ptclds, pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld_filtered);

    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const OctomapGeneratorAction::Goal> goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleOctomapGenerator> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandleOctomapGenerator> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleOctomapGenerator> goal_handle);
  };
}

#endif //COMPOSE_ITEMS_H
