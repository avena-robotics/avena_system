#include <helpers_vision/helpers_vision.hpp>
#include <string>
#include "pcl/point_types_conversion.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

// __CPP__
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/client_goal_handle.hpp"

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/distances.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <custom_interfaces/action/hand_eye.hpp>
#include <cmath>
#include <sensor_msgs/msg/camera_info.hpp>

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include "helpers_commons/helpers_commons.hpp"
#include <custom_interfaces/action/generate_path_pose.hpp>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "helpers_vision/visualization.hpp" 


// #define BOARD_BOX_SIZE 0.0417
#define BOARD_BOX_SIZE 0.04025
#define CHESS_BOARD_PATTERN_WIDTH 4
#define CHESS_BOARD_PATTERN_HIGHT 3
#define AMOUNT_SAMPLES_PER_CAMERA 5


#define PANDA_BASE_LINK "right_franka_link_0"
#define PANDA_GRIPPER_LINK "right_franka_gripper_connection"
#define PANDA_EE_LINK "right_franka_calibration_mat"

using namespace std::chrono_literals;
using action_resault = std::shared_future<rclcpp_action::ClientGoalHandle<custom_interfaces::action::SimpleAction>::WrappedResult>;

using HandEyeAction = custom_interfaces::action::HandEye;
using GoalHandleHandEye = rclcpp_action::ServerGoalHandle<HandEyeAction>;


struct P2P
{
    P2P(pcl::PointXYZ p1, pcl::PointXYZ p2) : p1(p1), p2(p2)
    {
        distance = pcl::euclideanDistance(p1, p2);
    }
    pcl::PointXYZ p1;
    pcl::PointXYZ p2;
    float distance;
};

class PclCalibrator : public rclcpp::Node
{

public:
    PclCalibrator(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    int calibrate(std::string &camera_rgb_topic, std::string &cam_info_topic, Eigen::Affine3f &transform);

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
    void saveToYaml(Eigen::Affine3f &camera_transform, std::string parent, std::string child, std::string filename);
    void publishTransform(const Eigen::Affine3f &in_transform, const std::string &parent, const std::string &child, bool is_static = false);
    void _displayTransform(const Eigen::Affine3f &in_transform, const std::string &parent, const std::string &child);
    int _move_robot(Eigen::Affine3f &pose);
    int _lookupTransform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &timestamp, Eigen::Affine3f &out_transform);
    void _publishRobotToTargerTF();
    int _validate_response(action_resault &result, std::string &action_name);
    int _call_simple_action(std::string name, std::chrono::seconds timeout);
    int _initiate_calibration();
    // void _joinWhenFinished();
    int _call_generate_path(std::string name, std::chrono::seconds timeout, Eigen::Affine3f &pose);

    void _rotate_samples(std::vector<Eigen::Affine3f> &input_samples, std::vector<Eigen::Affine3f> &output_samples);



    bool _calibration_finished = false;
    helpers::SubscriptionsManager::SharedPtr _subscriptions_manager;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> _static_broadcaster;
    rclcpp::TimerBase::SharedPtr _robot_to_target_tf_timer;
    std::unique_ptr<tf2_ros::TransformListener> _transform_listener;
    std::unique_ptr<tf2_ros::Buffer> _transforms_buffer;

    bool _display_cv = false;

    std::string _generate_path_name = "generate_path_pose"; //TODO change this name to valid
    std::string _generate_trajectory_name = "generate_trajectory";
    std::string _execute_move_name = "execute_move";
    std::string _camera1_rgb_topic = "/camera_1/rgb/image_raw";
    std::string _camera2_rgb_topic = "/camera_2/rgb/image_raw";
    std::string _camera1_info_topic = "/camera_1/rgb/camera_info";
    std::string _camera2_info_topic = "/camera_2/rgb/camera_info";

    std::vector<Eigen::Affine3f> _cam1_robot_positions;
    std::vector<Eigen::Affine3f> _cam2_robot_positions;

    void _joinWhenFinished();
    void _waitForKeyPress();
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





};