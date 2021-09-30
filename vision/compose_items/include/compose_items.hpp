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

#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <nlohmann/json.hpp>
#include "pcl_methods/create_ptcld.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

//ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "custom_interfaces/action/simple_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "custom_interfaces/msg/detections_list.hpp"
#include "custom_interfaces/msg/items.hpp"
#include "custom_interfaces/msg/images.hpp"

#include "custom_interfaces/srv/data_store_detectron_select.hpp"
#include "custom_interfaces/srv/data_store_rgbd_sync_select.hpp"
#include "custom_interfaces/srv/data_store_items_insert.hpp"
#include "custom_interfaces/srv/data_store_items_select.hpp"

using namespace std::chrono_literals;
using camera_id = size_t;
using label = std::string;

#define log_debug std::cout << __func__ << " " << __LINE__ << std::endl;
 
namespace compose_items
{

  using json = nlohmann::json;
  using Response = custom_interfaces::srv::DataStoreItemsSelect::Response;

  typedef std::chrono::system_clock::time_point TimeVar;
#define duration(a) std::chrono::duration_cast<std::chrono::milliseconds>(a).count()
#define timeNow() std::chrono::high_resolution_clock::now()

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
    int _logWarn(std::string msg);
    int _readLabels();
    int _assignData(custom_interfaces::msg::DetectionsList::SharedPtr &detect_msg, custom_interfaces::msg::Images::SharedPtr &depth_images, custom_interfaces::msg::Images::SharedPtr &rgb_images);
    int _validateInputs(custom_interfaces::msg::DetectionsList::SharedPtr &detect_msg, custom_interfaces::msg::Images::SharedPtr &depth_images_msg, custom_interfaces::msg::Images::SharedPtr &rgb_images_msg);
    void _readAreasParameters();
    void _getCamerasParameters();
    int _sendDataToDB(Response::SharedPtr &compose_msg);
    void _computeElementData(std::string label, size_t el_idx, size_t cam_idx);
    bool _initial_match(element &cam1_el, element &el);
    void _assignParent(element & parent, element & child);
    void _findAllChilds(label label ,size_t parent_cam_idx, size_t child_cam_idx);
    void _matchDetectionsForEachCamera();

//     element* _getParent(size_t &cam_idx, size_t &item_id, std::string label);
// element* _getParentItems(size_t &cam_idx,size_t &item_id = 0, std::map<label, std::vector<element>>::iterator &item_label_it);
void _matchElementsWithItem(element& parent);
  void _assignElements();
    void _passAllUnmatchedDetections();



    bool _unpackMap(size_t &parent_cam_idx, size_t &child_cam_idx, std::string &label);

    std::optional<std::pair<element*, element* >> _getParentChildPair(size_t parent_cam_idx,size_t child_cam_idx, std::string label );

    std::vector<std::string> _labels;
    std::map<std::string, bool> _items;
    std::map<std::string, bool> _elements;
    std::map<std::string, std::vector<std::string>> _components;
    WorkspaceArea _workspace_area;

    // //ROS
    rclcpp::Client<custom_interfaces::srv::DataStoreDetectronSelect>::SharedPtr _detectron_client;
    rclcpp::Client<custom_interfaces::srv::DataStoreRgbdSyncSelect>::SharedPtr _rgbd_sync_client;
    rclcpp::Client<custom_interfaces::srv::DataStoreItemsInsert>::SharedPtr _items_client;

    rclcpp_action::Server<ComposeItemsAction>::SharedPtr _action_server;
    custom_interfaces::msg::DetectionsList::SharedPtr _detect_msg;
    custom_interfaces::msg::Images::SharedPtr _depth_images_msg;
    custom_interfaces::msg::Images::SharedPtr _rgb_images_msg;

    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ComposeItemsAction::Goal> goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleComposeItems> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandleComposeItems> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleComposeItems> goal_handle);
    void _save_data(custom_interfaces::msg::Items &items);

    size_t _cameras_amount = 0;
    camera_data_ptr _cameras_data;
    std::map<size_t, std::shared_ptr<cv::Mat>> depths;
    std::map<size_t, std::shared_ptr<cv::Mat>> rgbs;

    std::map<camera_id, std::map<label, std::vector<element>>> _items_data;

    std::map<camera_id, std::map<label, std::vector<element>>> _elements_data;

    bool _remove_shadows;
    std::map<element *, std::vector<element *>> _matched_items;
    
    std::map<element *, std::vector<element *>> _matched_elements;

    
    size_t item_id = 0;
    size_t element_id = 0;

    // std::map<std::string, size_t> debug_map;

  };
}

#endif //COMPOSE_ITEMS_H
