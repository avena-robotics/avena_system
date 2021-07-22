#include <helpers_vision/helpers_vision.hpp>
#include <string>
#include "pcl/point_types_conversion.h"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/distances.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <chrono>
// #include <visualization.hpp>

#define SMALLER_DISTANCE 0.15
#define BIGGER_DISTANCE 0.23
#define CIRCLE_RADIUS 0.025

#define AMOUNT_OF_SAMPLES 5

using namespace std::chrono_literals;
using iterator = std::vector<pcl::PointXYZHSV, Eigen::aligned_allocator<pcl::PointXYZHSV>>::iterator;

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



class Calibrator : public rclcpp::Node
{


public:
    Calibrator();



private:
    const int RED = 0;
    const int YELLOW = 60;
    const int LIME = 120;
    const int BLUE = 240;
    const int CYAN = 180;
    const int MAGENTA = 300;
    const int THRESH = 29;

    std::thread main_logic_thread;

    std::vector<Eigen::Affine3f> _cam2cam_transforms;
    sensor_msgs::msg::PointCloud2::SharedPtr camera1_pcd;
    sensor_msgs::msg::PointCloud2::SharedPtr camera2_pcd;
    bool stop_saving_samples_camera_1;
    bool stop_saving_samples_camera_2;
    std::shared_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_camera1_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_camera2_;
    std::vector<Eigen::Matrix4f> samples_camera_1;
    std::vector<Eigen::Matrix4f> samples_camera_2;

    bool _can_join;
    rclcpp::TimerBase::SharedPtr _join_check_timer;

    void _execute();
    void _publishAvg(Eigen::Affine3f &camera_1_to_base, Eigen::Affine3f &camera_2_to_base);
    void saveToYaml(Eigen::Affine3f &camera_transform, std::string parent, std::string child, std::string filename);
    void findPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void findPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr plane_coefficients);
    void publishTransform(const Eigen::Affine3f &in_transform, const std::string &parent, const std::string &child);
    void findTransformFromPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Affine3f &out_trans, std::string camera_name);
    void _displayTransform(const Eigen::Affine3f &in_transform, const std::string &parent, const std::string &child);
    void camera_1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void camera_2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    
    void _waitForKeyPress();
    void _joinWhenFinished();


};
