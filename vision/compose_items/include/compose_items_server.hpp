#ifndef COMPOSE_ITEMS_H
#define COMPOSE_ITEMS_H

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
// #include <mysql_connector.h>
// #include <mysql_connector_structs.h>

// #include "utilities.h"
#include <nlohmann/json.hpp>
#include "pcl_methods/create_ptcld.hpp"
#include "structures.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

//ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "custom_interfaces/action/simple_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "custom_interfaces/msg/detections.hpp"
#include "custom_interfaces/msg/depth_images.hpp"
#include "custom_interfaces/msg/items.hpp"
#include "custom_interfaces/msg/rgb_images.hpp"
#include "custom_interfaces/srv/data_store_detectron_select.hpp"
#include "custom_interfaces/srv/data_store_rgbd_sync_select.hpp"
// #include "custom_interfaces"

using namespace std::chrono_literals;

namespace compose_items
{

  using json = nlohmann::json;

  typedef std::chrono::system_clock::time_point TimeVar;
#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define timeNow() std::chrono::high_resolution_clock::now()

  // using namespace ;

  class ComposeItems : public rclcpp::Node, public helpers::WatchdogInterface
  {
  public:
    using ComposeItemsAction = custom_interfaces::action::SimpleAction;
    using GoalHandleComposeItems = rclcpp_action::ServerGoalHandle<ComposeItemsAction>;

    using transform_map = std::map<std::string, Eigen::Affine3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Affine3f>>>;

    ComposeItems(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), bool debug = false);
    ~ComposeItems();

    virtual void initNode() override;
    virtual void shutDownNode() override;

    void composeItemsData();

  private:
    helpers::Watchdog::SharedPtr _watchdog;
    void _getData();

    int _readLabels();
    int _assignData(custom_interfaces::msg::Detections::SharedPtr &detect_msg, custom_interfaces::msg::DepthImages::SharedPtr &depth_images_msg);
    int _assignElement(std::string label, detected_item_t &item, std::vector<element_t> &out_elements);
    int _assignItem(uint32_t camera_index, item_cam_t &detection, detected_item_t &item);
    int _getItemsStartingIdx();
    // int _createScenePointClouds();
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> _checkPtcld(std::shared_ptr<item_cam_t> item_cam_ptr);
    std::shared_ptr<cv::Mat> _getMaskPtr(std::shared_ptr<item_cam_t> item_cam_ptr);
    std::shared_ptr<cv::Mat> _getDepthPtr(std::shared_ptr<item_cam_t> item_cam_ptr);
    bool _isComponent(detected_item_t &item, element_t &element, const std::vector<element_t> &elements);
    bool _checkOverlay(cv::Mat &item_mask, cv::Mat &element_mask);
    int _validateInputs(custom_interfaces::msg::Detections::SharedPtr &detect_msg, custom_interfaces::msg::DepthImages::SharedPtr &depth_images_msg);
    void _readAreasParameters();


    // _readSceneData();
    uint32_t _starting_index = 0;
    // uint32_t _db_item_offset;
    uint32_t _element_pk = 0;
    std::vector<uint32_t> _matched_cam2_indices;

    std::vector<detected_item_t> _items_to_save;
    std::vector<detected_item_t> _remaining_items;
    std::vector<element_t> _elements_to_save;

    // std::shared_ptr<MysqlConnector> _sql_connector;
    std::vector<std::string> _labels;
    // std::map<std::string, uint32_t> _labels_str_to_int;

    std::map<std::string, bool> _items;
    std::map<std::string, bool> _elements;
    std::map<std::string, std::vector<std::string>> _components;
    std::shared_ptr<current_scene_t> _scene;
    float _leaf_size;
    std::shared_ptr<robot::CreatePtcld> _create_ptcld;

    std::vector<item_cam_t> _detections_cam1;
    std::vector<item_cam_t> _detections_cam2;
    bool _debug;
    builtin_interfaces::msg::Time _last_rgb_images_processed_msg_timestamp;
    builtin_interfaces::msg::Time _last_depth_images_processed_msg_timestamp;
    builtin_interfaces::msg::Time _last_detections_processed_msg_timestamp;


    WorkspaceArea _workspace_area;

    //ROS
    rclcpp::Client<custom_interfaces::srv::DataStoreDetectronSelect>::SharedPtr _detectron_client;
    rclcpp::Client<custom_interfaces::srv::DataStoreRgbdSyncSelect>::SharedPtr _rgbd_sync_client;

    rclcpp_action::Server<ComposeItemsAction>::SharedPtr _action_server;
    rclcpp::Publisher<custom_interfaces::msg::Items>::SharedPtr _publisher;
    rclcpp::Subscription<custom_interfaces::msg::Detections>::SharedPtr _new_masks_subscriber;
    rclcpp::Subscription<custom_interfaces::msg::RgbImages>::SharedPtr _rgb_images_subscriber;
    rclcpp::Subscription<custom_interfaces::msg::DepthImages>::SharedPtr _depth_images_subscriber;

    custom_interfaces::msg::Detections::SharedPtr _detect_msg;
    custom_interfaces::msg::DepthImages::SharedPtr _depth_images_msg;
    custom_interfaces::msg::RgbImages::SharedPtr _rgb_images_msg;

    void _getCamerasParameters();
    int _saveComposedData(custom_interfaces::msg::Items::UniquePtr &compose_msg);
    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ComposeItemsAction::Goal> goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleComposeItems> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandleComposeItems> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleComposeItems> goal_handle);
    void _checkLastMessagesTimestamps();
    void _initializeSubscribers(const rclcpp::QoS &qos_settings);
  };
}

#endif //COMPOSE_ITEMS_H
