#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp/rclcpp.hpp>

#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES

class echoListener
{
public:
    tf2_ros::Buffer buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfl_;

    explicit echoListener(rclcpp::Clock::SharedPtr clock)
        : buffer_(clock)
    {
        tfl_ = std::make_shared<tf2_ros::TransformListener>(buffer_);
    }

    ~echoListener() = default;
};

// Util command line argument parser
class CmdOptionsParser
{
public:
    explicit CmdOptionsParser(int argc, char **argv)
    {
        for (int i = 0; i < argc; i++)
            _cmd_args.push_back(argv[i]);
    }

    bool optionExists(const std::string &option)
    {
        return std::find(_cmd_args.begin(), _cmd_args.end(), option) != _cmd_args.end();
    }

    std::string getCmdOption(const std::string &option)
    {
        auto it = std::find(_cmd_args.begin(), _cmd_args.end(), option) + 1;
        if (it == _cmd_args.end())
            return "";
        return *it;
    }

private:
    std::vector<std::string> _cmd_args;
};

int main(int argc, char **argv)
{
    // Initialize ROS
    std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf2_echo");
    
    // Setup constants and parsed variables
    constexpr double RAD_TO_DEG = 180.0 / M_PI;
    double rate_hz = 1.0;
    size_t precision = 3;

    const std::string help = "Usage: tf2_echo source_frame target_frame [precision \"precision\"] [rate \"echo_rate\"]\n\n"
                             "This will echo the transform from the coordinate frame of the \"source_frame\"\n"
                             "to the coordinate frame of the \"target_frame\".\n"
                             "Note: This is the transform to get data from \"target_frame\" into the \"source_frame\".\n"
                             "Default echo rate is 1 if echo_rate is not given.\n"
                             "Default decimal places precision is 3.\n";

    if (args.size() < 3)
    {
        RCLCPP_ERROR_STREAM(nh->get_logger(), "Source and target frames are required" << std::endl << help);
        return 1;
    }

    CmdOptionsParser cmd_parser(argc, argv);
    try
    {
        if (cmd_parser.optionExists("rate"))
        {
            std::string rate_str = cmd_parser.getCmdOption("rate");
            if (!rate_str.empty())
                rate_hz = std::stod(rate_str);
        }

        if (cmd_parser.optionExists("precision"))
        {
            std::string precision_str = cmd_parser.getCmdOption("precision");
            if (!precision_str.empty())
                precision = std::stoi(precision_str);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(nh->get_logger(), "Cannot parse command line arguments" << std::endl
                                                                                    << help);
        return 1;
    }

    rclcpp::Rate rate(rate_hz);

    rclcpp::Clock::SharedPtr clock = nh->get_clock();
    // Instantiate a local listener
    echoListener echoListener(clock);

    std::string source_frameid = args[1];
    std::string target_frameid = args[2];

    // Wait for the first transforms to become avaiable.
    std::string warning_msg;
    while (rclcpp::ok() && !echoListener.buffer_.canTransform(
                               source_frameid, target_frameid, tf2::TimePoint(), &warning_msg))
    {
        RCLCPP_WARN_THROTTLE(
            nh->get_logger(), *clock, 1000, "Waiting for transform %s ->  %s: %s",
            source_frameid.c_str(), target_frameid.c_str(), warning_msg.c_str());
        rate.sleep();
    }

    // Nothing needs to be done except wait for a quit
    // The callbacks within the listener class will take care of everything
    while (rclcpp::ok())
    {
        try
        {
            geometry_msgs::msg::TransformStamped echo_transform;
            echo_transform = echoListener.buffer_.lookupTransform(
                source_frameid, target_frameid,
                tf2::TimePoint());
            std::cout.precision(precision);
            std::cout.setf(std::ios::fixed, std::ios::floatfield);
            std::cout << "At time " << echo_transform.header.stamp.sec << "." << echo_transform.header.stamp.nanosec << std::endl;

            auto translation = echo_transform.transform.translation;
            auto rotation = echo_transform.transform.rotation;
            // Convert quaternion to RPY
            tf2::Quaternion quat(rotation.x, rotation.y, rotation.z, rotation.w);
            tf2::Matrix3x3 m(quat);
            double yaw, pitch, roll;
            m.getRPY(roll, pitch, yaw);
            std::cout << "Source frame: \"" << source_frameid << "\", target frame: \"" << target_frameid << "\"" << std::endl;
            std::cout << "- Translation: [" << translation.x << ", " << translation.y << ", " << translation.z << "]" << std::endl;
            std::cout << "- Rotation: in Quaternion [" << rotation.x << ", " << rotation.y << ", " << rotation.z << ", " << rotation.w << "]" << std::endl;
            std::cout << "- Rotation: in RPY (radian) [" << roll << ", " << pitch << ", " << yaw << "]" << std::endl;
            std::cout << "- Rotation: in RPY (degree) [" << roll * RAD_TO_DEG << ", " << pitch * RAD_TO_DEG << ", " << yaw * RAD_TO_DEG << "]" << std::endl;

            std::cout << "---" << std::endl;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_ERROR_STREAM(nh->get_logger(),
                                "Failure at " << clock->now().seconds() << std::endl
                                              << "Exception thrown:" << ex.what() << std::endl
                                              << "The current list of frames is:" << std::endl
                                              << echoListener.buffer_.allFramesAsString());
        }
        rate.sleep();
    }

    return 0;
}