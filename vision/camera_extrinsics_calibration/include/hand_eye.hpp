#ifndef CAMERA_EXTRINSICS_CALIBRATION__HAND_EYE_HPP_
#define CAMERA_EXTRINSICS_CALIBRATION__HAND_EYE_HPP_

#include "pcl/point_types_conversion.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// ___CPP___
#include <string>
#include <cmath>
#include <boost/filesystem.hpp>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <stdio.h>
#include <iostream>
#include <mutex>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ___OpenCV___
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

// ___Avena___
#include "helpers_vision/helpers_vision.hpp"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/visualization.hpp"
#include "custom_interfaces/action/simple_action.hpp"
#include "custom_interfaces/action/generate_path_pose.hpp"
#include "custom_interfaces/srv/change_tool.hpp"


// #define BOARD_BOX_SIZE 0.0417
#define BOARD_BOX_SIZE 0.04025
#define CHESS_BOARD_PATTERN_WIDTH 4
#define CHESS_BOARD_PATTERN_HIGHT 3
#define AMOUNT_SAMPLES_PER_CAMERA 10

#define PANDA_BASE_LINK "right_franka_link_0"
#define WORLD "world"
#define PANDA_GRIPPER_LINK "right_franka_gripper_connection"
#define PANDA_EE_LINK "right_franka_calibration_mat"

constexpr uint8_t MIN_PIXEL_VAL = 0;
constexpr uint8_t MAX_PIXEL_VAL = 255;

using namespace std::chrono_literals;
using ActionResult = std::shared_future<rclcpp_action::ClientGoalHandle<custom_interfaces::action::SimpleAction>::WrappedResult>;
using HandEyeAction = custom_interfaces::action::SimpleAction;
using GoalHandleHandEye = rclcpp_action::ServerGoalHandle<HandEyeAction>;

enum CalibrateReturnCode
{
    SUCCESS,
    CAMERA_INFO_ERROR,
    IMAGE_ERROR,
    WORLD_TO_END_EFFECTOR_TF_ERROR,
    CAMERA_BASE_TO_RGB_LINK_TF_ERROR,
    CALIBRATION_MAT_NOT_FOUND,
};

class PclCalibrator : public rclcpp::Node
{

public:
    PclCalibrator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    CalibrateReturnCode calibrate(std::string &camera_rgb_topic, std::string &cam_info_topic, Eigen::Affine3f &out_transform, int &in_out_curr_threshold);

    template <typename ActionT>
    int waitForServer(const typename rclcpp_action::Client<ActionT>::SharedPtr action_client)
    {
        size_t wait_time = 60;
        size_t seconds_waited = 0;
        while (seconds_waited < wait_time)
        {
            if (!action_client->wait_for_action_server(1s))
            {
                RCLCPP_ERROR_STREAM(this->get_logger(), "Action server not available after waiting for " << ++seconds_waited << " seconds.");
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "Problem with ROS. Exiting...");
                    throw std::runtime_error("Problem with ROS. Exiting...");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Action server avaiable.");
                break;
            }
        }
        if (seconds_waited == wait_time)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Action server not available. Exiting...");
            return 1;
        }
        return 0;
    }

private:
    void _saveToYaml(Eigen::Affine3f &camera_transform, std::string parent, std::string child, std::string filename);
    void _publishTransform(const Eigen::Affine3f &in_transform, const std::string &parent, const std::string &child, bool is_static = false);
    void _displayTransform(const Eigen::Affine3f &in_transform, const std::string &parent, const std::string &child);
    int _moveRobot(Eigen::Affine3f &pose);
    int _lookupTransform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &timestamp, Eigen::Affine3f &out_transform);
    void _publishRobotToTargerTF();
    int _validateResponse(ActionResult &result, std::string &action_name);
    int _callSimpleAction(std::string name, std::chrono::seconds timeout);
    int _initiateCalibration();
    // void _joinWhenFinished();
    int _callGeneratePath(std::string name, std::chrono::seconds timeout, Eigen::Affine3f &pose);
    void _rotateSamples(std::vector<Eigen::Affine3f> &input_samples, const size_t rotations_per_sample, std::vector<Eigen::Affine3f> &output_samples);
    void _initializeSubscribers(const rclcpp::QoS &qos_settings);
    std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr> _getData(const std::string &rgb_image_topic, const std::string &camera_info_topic);
    int _attachCalibrationMat();
    int _detachCalibrationMat();
    int _changeTool(const std::string &tool_name);
    
    int _goHome();

    // helpers::SubscriptionsManager::SharedPtr _subscriptions_manager;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _static_broadcaster;
    rclcpp::TimerBase::SharedPtr _robot_to_target_tf_timer;
    std::unique_ptr<tf2_ros::TransformListener> _transform_listener;
    std::unique_ptr<tf2_ros::Buffer> _transforms_buffer;
    rclcpp::Client<custom_interfaces::srv::ChangeTool>::SharedPtr _change_tool_client;

    // Images data subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _camera1_rgb_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _camera2_rgb_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera1_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera2_info_sub;

    sensor_msgs::msg::Image::SharedPtr _camera1_rgb_image;
    sensor_msgs::msg::Image::SharedPtr _camera2_rgb_image;
    sensor_msgs::msg::CameraInfo::SharedPtr _camera1_info;
    sensor_msgs::msg::CameraInfo::SharedPtr _camera2_info;

    std::mutex _camera1_rgb_image_mtx;
    std::mutex _camera2_rgb_image_mtx;
    std::mutex _camera1_info_mtx;
    std::mutex _camera2_info_mtx;

    bool _display_cv = false;

    std::vector<Eigen::Affine3f> _cam1_robot_positions;
    std::vector<Eigen::Affine3f> _cam2_robot_positions;

    int _camera_index;

    //ros action
    rclcpp_action::Server<HandEyeAction>::SharedPtr _action_server;
    rclcpp_action::GoalResponse _handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const HandEyeAction::Goal> goal);
    rclcpp_action::CancelResponse _handleCancel(const std::shared_ptr<GoalHandleHandEye> goal_handle);
    void _handleAccepted(const std::shared_ptr<GoalHandleHandEye> goal_handle);
    void _execute(const std::shared_ptr<GoalHandleHandEye> goal_handle);

    std::thread main_logic_thread;
    rclcpp::TimerBase::SharedPtr _join_check_timer;

    std::vector<Eigen::Affine3f> _cam1_to_base_samples;
    std::vector<Eigen::Affine3f> _cam2_to_base_samples;

    std::string _generate_path_name = "generate_path_pose";
    std::string _generate_path_home = "generate_path_home";
    std::string _generate_trajectory_name = "generate_trajectory";
    std::string _execute_move_name = "execute_move";
    std::string _camera1_rgb_topic = "camera_1/rgb/image_raw";
    std::string _camera2_rgb_topic = "camera_2/rgb/image_raw";
    std::string _camera1_base = "camera_1/camera_base";
    std::string _camera2_base = "camera_2/camera_base";
    std::string _camera1_info_topic = "camera_1/rgb/camera_info";
    std::string _camera2_info_topic = "camera_2/rgb/camera_info";

    const std::string CALIBRATION_MAT_NAME = "calibration_mat";
    const std::string GRIPPER_NAME = "gripper";
};

#endif // CAMERA_EXTRINSICS_CALIBRATION__HAND_EYE_HPP_
