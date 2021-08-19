#ifndef OCTOMAP_FILTER_HPP_
#define OCTOMAP_FILTER_HPP_
// __CPP__
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <algorithm>
#include <optional>

// ___PCL___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/octree/octree_pointcloud_adjacency.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/common/distances.h>
#include <pcl/filters/conditional_removal.h>
// ___Avena___
// #include "custom_interfaces/msg/octomap_filter.hpp"
#include "custom_interfaces/msg/filtered_scene_octomap.hpp"
// #include "custom_interfaces/msg/scene.hpp"
// #include "custom_interfaces/msg/estimate_shape.hpp"
#include "custom_interfaces/action/simple_action.hpp"
// #include "custom_interfaces/msg/item_select.hpp"
#include "custom_interfaces/msg/items.hpp"
#include "custom_interfaces/msg/merged_ptcld_filtered.hpp"
#include "custom_interfaces/msg/selected_items_ids.hpp"
#include "visibility_control.h"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"

namespace octomap_filter
{
    using json = nlohmann::json;

    struct DishData
    {
        float lower_disc_radius;
        float lower_disc_height;
        float middle_disc_radius;
        float middle_disc_height;
        float upper_disc_radius;
        float upper_disc_height;
        float base_height;
        // lower rant angle
        float alpha;
        // upper rant angle
        float beta;
    };
    struct KnifeData
    {
        Eigen::Vector3f dimensions_blade;
        Eigen::Vector3f dimensions_handle;
        Eigen::Vector3f offset_blade;
        Eigen::Vector3f offset_handle;
    };

    struct SpatulaData
    {
        Eigen::Vector3f dimensions_spatula_tool;
        Eigen::Vector3f dimensions_handle;
        Eigen::Vector3f dimensions_connector;
        Eigen::Vector3f offset_spatula_tool;
        Eigen::Vector3f offset_handle;
        Eigen::Vector3f offset_connector;
    };

    struct Label
    {
        // int32_t label_id = -1;
        std::string label{};
        std::string fit_method{};
        json fit_method_parameters{};
        bool item = false;
        bool element = false;
        json components{};
        json item_description;
    };

    class OctomapFilter : public rclcpp::Node, public helpers::WatchdogInterface
    {
    public:
        // Action
        using OctomapFilterAction = custom_interfaces::action::SimpleAction;
        using GoalHandleOctomapFilter = rclcpp_action::ServerGoalHandle<OctomapFilterAction>;
        // Message
        // using OctomapFilterMsg = custom_interfaces::msg::OctomapFilter;
        // using InputMsg = custom_interfaces::msg::ItemSelect;

        OCTOMAP_FILTER_PUBLIC
        explicit OctomapFilter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

        void initNode() override;
        void shutDownNode() override;

        // explicit OctomapFilter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions(), bool debug = false);

    private:
        helpers::Watchdog::SharedPtr _watchdog;

        OCTOMAP_FILTER_LOCAL
        rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const OctomapFilterAction::Goal> goal);
        OCTOMAP_FILTER_LOCAL
        rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleOctomapFilter> goal_handle);
        OCTOMAP_FILTER_LOCAL
        void _handleAccepted(const std::shared_ptr<GoalHandleOctomapFilter> goal_handle);
        OCTOMAP_FILTER_LOCAL
        void _execute(const std::shared_ptr<GoalHandleOctomapFilter> goal_handle);
        // OCTOMAP_FILTER_LOCAL
        // void _itemSelectCallback(const custom_interfaces::msg::ItemSelect::SharedPtr item_select_msg);
        OCTOMAP_FILTER_LOCAL
        bool _voxelizePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr out_scene_filtered);
        OCTOMAP_FILTER_LOCAL
        custom_interfaces::msg::FilteredSceneOctomap::UniquePtr _prepareOutputMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered);
        OCTOMAP_FILTER_LOCAL
        void _readOctomapVoxelSizeFromParametersServer(std::vector<Label> &out_labels_parameters);
        OCTOMAP_FILTER_LOCAL
        void _getItemPtcld(pcl::PointCloud<pcl::PointXYZ>::Ptr item_pointcloud);
        OCTOMAP_FILTER_LOCAL
        int _validateInput();
        OCTOMAP_FILTER_LOCAL
        void _checkLastMessagesTimestamps(const builtin_interfaces::msg::Time &merged_items_stamp, const builtin_interfaces::msg::Time &merged_ptcld_filtered_stamp);
        OCTOMAP_FILTER_LOCAL
        void _initializeSubscribers(const rclcpp::QoS &qos_settings);
        OCTOMAP_FILTER_LOCAL
        void _getDistanceFromLine(Eigen::Vector3f point, Eigen::Vector3f line_origin, Eigen::Vector3f line_direction, float &distance);
        OCTOMAP_FILTER_LOCAL
        int _filterDish(DishData bowl_data, json item_information, Eigen::Affine3f item_transform, float buffor_radius, float buffor_height, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_scene_filtered);
        OCTOMAP_FILTER_LOCAL
        int _cropBox(const Eigen::Affine3f &transform, const Eigen::Vector3f dimensions, bool outside_of_the_box, pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld);

        // // ROS
        rclcpp_action::Server<OctomapFilterAction>::SharedPtr _action_server;
        rclcpp::Subscription<custom_interfaces::msg::MergedPtcldFiltered>::SharedPtr _subscriber_merged_ptcld_filtered;
        rclcpp::Subscription<custom_interfaces::msg::Items>::SharedPtr _subscriber_merged_items;
        custom_interfaces::msg::MergedPtcldFiltered::SharedPtr _merged_ptcld_filtered_msg;
        custom_interfaces::msg::Items::SharedPtr _items_msg;
        rclcpp::Publisher<custom_interfaces::msg::FilteredSceneOctomap>::SharedPtr _publisher_octomap_filter;

        // custom_interfaces::msg::ItemSelect _item_select_data;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr _merged_point_cloud;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr _octomap_data_filtered;
        std::vector<Label> _out_labels_parameters;
        float _grid_size;
        builtin_interfaces::msg::Time _last_merged_items_msg_timestamp;
        builtin_interfaces::msg::Time _last_merged_ptcld_filtered_msg_timestamp;
    };
} // namespace octomap_filter

#endif //OCTOMAP_FILTER_HPP_
