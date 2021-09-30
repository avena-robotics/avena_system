#ifndef ROBOT_SELF_FILTER_HPP
#define ROBOT_SELF_FILTER_HPP

// ___CPP___
#include <functional>
#include <memory>

// __ROS__
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <tf2_ros/transform_listener.h>

// __PCL__
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>

// ___Avena___
#include "robot_self_filter/visibility_control.h"
#include "helpers_commons/helpers_commons.hpp"
#include "helpers_vision/helpers_vision.hpp"
#include "helpers_commons/structures.hpp"

namespace robot_self_filter
{

    struct AvenaMesh
    {
        AvenaMesh()
            : cloud(new pcl::PointCloud<pcl::PointXYZ>)
        {
        }
        pcl::PointXYZ min_pt;
        pcl::PointXYZ max_pt;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        pcl::PolygonMesh mesh;
    };

    class RobotSelfFilter 
    {
    public:
        COMPOSITION_PUBLIC
        RobotSelfFilter(rclcpp::Node *node, float &offset);
    void removeRobotFromCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr merged_ptcld);

          

    private:
        helpers::Watchdog::SharedPtr _watchdog;
        rclcpp::Node *_node;
        void _loadAvenaMeshes();
        void _removeRobotFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud);
        std::vector<std::string> _removeRobotPrefix(std::vector<std::string> link_names, std::string robot_prefix);

        /**
         * @brief Get the transform between two frames by frame ID. If function return value is 1 - there is no valid transform between frames - this cloud affect further logic so be sure to handle it!
         * 
         * @param target_frame The frame to which data should be transformed
         * @param source_frame The frame where the data originated
         * @param timestamp Timestamp at which transform should be taken
         * @param out_transform Result transform of robot link
         * @return int Return code - 0 for success, 1 for failure
         */

        int _lookupTransform(const std::string &target_frame, const std::string &source_frame, const std::chrono::microseconds &timestamp, Eigen::Affine3f &out_transform);

       std::vector<std::string> _robot_links_names;
        std::map<std::string, AvenaMesh> _meshes;
        std::unique_ptr<tf2_ros::TransformListener> _transform_listener;
        std::unique_ptr<tf2_ros::Buffer> _transforms_buffer;

        helpers::commons::RobotInfo _robot_info;
        std::string _robot_prefix;

        float _offset;
        bool _debug;
    };

} // namespace robot_self_filter

#endif
