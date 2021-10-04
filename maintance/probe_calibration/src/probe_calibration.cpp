#include "probe_calibration/probe_calibration.hpp"

namespace probe_calibration
{
    ProbeCalibration::ProbeCalibration(const rclcpp::NodeOptions &options)
        : Node("probe_calibration", options)
    {
        std::thread([this]()
                    {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        while (rclcpp::ok())
                        {
                            if (!_readingParamsCallback())
                                break;
                            std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        }
                    })
            .detach();

        _probe_calibration_action_server = rclcpp_action::create_server<ProbeCalibrationAction>(
            this,
            "probe_calibration",
            [this](const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ProbeCalibrationAction::Goal> /*goal*/) -> rclcpp_action::GoalResponse
            {
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            },
            [this](const std::shared_ptr<GoalHandleProbeCalibration> /*goal_handle*/) -> rclcpp_action::CancelResponse
            {
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this](const std::shared_ptr<GoalHandleProbeCalibration> goal_handle)
            {
                std::thread(std::bind(&ProbeCalibration::_calibrateWithProbe, this, std::placeholders::_1), goal_handle).detach();
            });

        _transforms_buffer = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::Duration(std::chrono::seconds(5)));
        _transform_listener = std::make_unique<tf2_ros::TransformListener>(*_transforms_buffer);
        _static_transform_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        // Get robot probe frame name
        if (auto robot_info_opt = helpers::commons::getRobotInfo())
        {
            _probe_frame_name = robot_info_opt->robot_prefix + "_probe";
            _base_frame_name = robot_info_opt->robot_prefix + "_link_0";
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Cannot read robot info. Exiting...");
            rclcpp::shutdown();
            std::exit(1);
        }
    }

    void ProbeCalibration::_calibrateWithProbe(const std::shared_ptr<GoalHandleProbeCalibration> goal_handle)
    {
        auto result = std::make_shared<ProbeCalibrationAction::Result>();

        if (_chessboard_square_size == -1 || _marker_frame_name.empty())
        {
            RCLCPP_ERROR(get_logger(), "Parameters are not read from the server. Aborting...");
            goal_handle->abort(result);
            return;
        }

        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rotation_pairs;
        for (size_t i = 0; i < 3; i++)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Reading " << i + 1 << " sample");
            std::thread key_pressed_thread([]()
                                           {
                                               std::cout << "Press enter to take a sample..." << std::endl;
                                               std::cin.get();
                                           });
            key_pressed_thread.join();

            // Read TF from camera to calibration board.
            Eigen::Vector3d positions = Eigen::Vector3d::Zero();
            Eigen::Matrix3d orientation = Eigen::Matrix3d::Zero();
            double prev_time_sec = 0;
            size_t samples_amount = 0;
            RCLCPP_INFO(get_logger(), "Reading transform between camera and detected marker");
            while (samples_amount < 200)
            {
                if (auto camera_to_marker_tf_opt = _lookupTransform(_rgb_frame_name, _marker_frame_name))
                {
                    double time_sec = camera_to_marker_tf_opt->header.stamp.sec + camera_to_marker_tf_opt->header.stamp.nanosec / 1e9;
                    if (time_sec != prev_time_sec)
                    {
                        Eigen::Affine3d current_camera_to_marker_tf;
                        helpers::converters::geometryToEigenAffine(camera_to_marker_tf_opt->transform, current_camera_to_marker_tf);
                        positions += current_camera_to_marker_tf.translation();
                        orientation += current_camera_to_marker_tf.rotation();

                        prev_time_sec = time_sec;
                        samples_amount++;
                    }
                }

                if (samples_amount % 50 == 0)
                {
                    RCLCPP_INFO_STREAM(get_logger(), "Collected " << samples_amount << " samples");
                }
            }
            Eigen::Affine3d camera_to_marker_tf = Eigen::Translation3d(positions / samples_amount) * Eigen::Quaterniond(orientation / samples_amount);
            _displayTransform(camera_to_marker_tf, std::cout);

            // Read TF from base to probe tip.
            Eigen::Affine3d base_to_probe_tf;
            if (auto base_to_probe_tf_opt = _lookupTransform(_base_frame_name, _probe_frame_name))
            {
                helpers::converters::geometryToEigenAffine(base_to_probe_tf_opt->transform, base_to_probe_tf);
            }
            else
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Cannot read transform between \"" << _base_frame_name << "\" and \"" << _probe_frame_name << "\". Aborting...");
                goal_handle->abort(result);
                return;
            }
            _displayTransform(base_to_probe_tf, std::cout);

            Eigen::Vector3d offset = _chessboard_square_size * camera_to_marker_tf.rotation().col(1).normalized();
            Eigen::Vector3d marker_moved_to_probe = camera_to_marker_tf.translation() - offset;
            rotation_pairs.push_back({marker_moved_to_probe, base_to_probe_tf.translation()});
        }




        // // Magic starts here...

        // Eigen::Vector3d trans_robot = rotation_pairs[0].second;
        // Eigen::Vector3d trans_camera = rotation_pairs[0].first;

        // RCLCPP_INFO(get_logger(), "Processing samples");
        // for (size_t i = 0; i < rotation_pairs.size(); i++){
        //     rotation_pairs[i].first -= trans_camera;
        //     rotation_pairs[i].second -= trans_robot;
        // }

        // Eigen::Quaterniond rot1;
        // rot1.setFromTwoVectors((rotation_pairs[1].first), (rotation_pairs[1].second));
        // rotation_pairs[1].first = (rot1 * rotation_pairs[1].first);
        // rotation_pairs[2].first = (rot1 * rotation_pairs[2].first);

        // Eigen::Quaterniond rot_to_y;
        // rot_to_y.setFromTwoVectors((rotation_pairs[1].first), Eigen::Vector3d::UnitY());
        // rotation_pairs[2].first = (rot_to_y * rotation_pairs[2].first);
        // rotation_pairs[2].second = (rot_to_y * rotation_pairs[2].second);
        // Eigen::Vector3d camera_temp_no_y = rotation_pairs[2].first;
        // Eigen::Vector3d robot_temp_no_y = rotation_pairs[2].second;
        // camera_temp_no_y.y() = 0.0;
        // robot_temp_no_y.y() = 0.0;

        // double angle = std::atan2(robot_temp_no_y.cross(camera_temp_no_y).norm(), robot_temp_no_y.dot(camera_temp_no_y));
        // Eigen::Quaterniond rot2(helpers::vision::assignRotationMatrixAroundY(angle).cast<double>());

        // Eigen::Quaterniond final_rot = (rot_to_y.inverse() * rot2 * rot_to_y * rot1);
        // Eigen::Vector3d final_trans = trans_robot - (final_rot * trans_camera);
        // Eigen::Affine3d final_aff = Eigen::Translation3d(final_trans) * final_rot;

        // //and it ends here

        Eigen::Affine3d final_aff;
        _computeCameraPositionFromThreePoints(rotation_pairs[0].second, rotation_pairs[0].first,rotation_pairs[1].second, rotation_pairs[1].first,rotation_pairs[2].second, rotation_pairs[2].first ,final_aff);

        geometry_msgs::msg::Transform tf_ros;
        helpers::converters::eigenAffineToGeometry(final_aff, tf_ros);
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.transform = tf_ros;
        transform_stamped.child_frame_id = _rgb_frame_name;
        transform_stamped.header.frame_id = _base_frame_name;

        _displayTransform(transform_stamped, std::cout);

        _static_transform_broadcaster->sendTransform(transform_stamped);
        RCLCPP_INFO(get_logger(), "Successfully read transformation between camera and base");
        goal_handle->succeed(result);
    }


void ProbeCalibration::_computeCameraPositionFromThreePoints(Eigen::Vector3d robot_A, Eigen::Vector3d camera_A,Eigen::Vector3d robot_B, Eigen::Vector3d camera_B,Eigen::Vector3d robot_C, Eigen::Vector3d camera_C, Eigen::Affine3d &out_pose){

        robot_B -= robot_A; 
        robot_C -= robot_A; 
        camera_B -= camera_A; 
        camera_C -= camera_A; 

        Eigen::Quaterniond rot1;
        rot1.setFromTwoVectors((camera_B), (robot_B));
        camera_B = (rot1 * camera_B);
        camera_C = (rot1 * camera_C);

        Eigen::Quaterniond rot_to_y;
        rot_to_y.setFromTwoVectors((camera_B), Eigen::Vector3d::UnitY());
        camera_C = (rot_to_y * camera_C);
        robot_C = (rot_to_y * robot_C);
        camera_C.y() = 0.0;
        robot_C.y() = 0.0;

        double angle = std::atan2(robot_C.cross(camera_C).norm(), robot_C.dot(camera_C));
        Eigen::Quaterniond rot2(helpers::vision::assignRotationMatrixAroundY(angle).cast<double>());

        Eigen::Quaterniond final_rot = (rot_to_y.inverse() * rot2 * rot_to_y * rot1);
        Eigen::Vector3d final_trans = robot_A - (final_rot * camera_A);
        out_pose = Eigen::Translation3d(final_trans) * final_rot;

}


 void ProbeCalibration::_computeChosenPointPosition(Eigen::Affine3d &camera_to_marker_tf, cv::Point &point, Eigen::Vector3d &out_marker_moved_to_probe){
        Eigen::Vector3d x_offset = _chessboard_square_size * point.x * camera_to_marker_tf.rotation().col(0).normalized();
        Eigen::Vector3d y_offset = _chessboard_square_size * point.y * camera_to_marker_tf.rotation().col(1).normalized();
        out_marker_moved_to_probe = camera_to_marker_tf.translation() + x_offset;
        out_marker_moved_to_probe = camera_to_marker_tf.translation() + y_offset;
} 





    std::optional<geometry_msgs::msg::TransformStamped> ProbeCalibration::_lookupTransform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &timestamp)
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            rclcpp::Duration duration(std::chrono::milliseconds(1000));
            transform = _transforms_buffer->lookupTransform(target_frame, source_frame, timestamp, duration);
        }
        catch (const tf2::TransformException &err)
        {
            std::cout << err.what() << std::endl;
            RCLCPP_INFO(get_logger(), err.what());
            return std::nullopt;
        }
        return transform;
    }

    void ProbeCalibration::_displayTransform(const geometry_msgs::msg::TransformStamped &tf, std::ostream &out)
    {
        out << "Transform:" << std::endl;
        out << " - Frame ID: " << tf.header.frame_id << std::endl;
        out << " - Child frame ID: " << tf.child_frame_id << std::endl;
        out << " - Translation: " << tf.transform.translation.x << " " << tf.transform.translation.y << " " << tf.transform.translation.z << std::endl;
        out << " - Rotation: " << tf.transform.rotation.x << " " << tf.transform.rotation.y << " " << tf.transform.rotation.z << " " << tf.transform.rotation.w << std::endl;
    }

    void ProbeCalibration::_displayTransform(const Eigen::Affine3d &tf, std::ostream &out)
    {
        geometry_msgs::msg::TransformStamped tf_ros_stamped;
        helpers::converters::eigenAffineToGeometry(tf, tf_ros_stamped.transform);
        _displayTransform(tf_ros_stamped, out);
    }

    int ProbeCalibration::_readingParamsCallback()
    {
        auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(shared_from_this(), "pose_estimation");
        if (!parameters_client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(get_logger(), "Service not available after waiting for 1 second. Trying again...");
            return -1;
        }

        std::string image_topic_name;
        try
        {
            auto params_future = parameters_client->get_parameters({"chessboard_square_size", "marker_frame_name", "camera_frame"});
            auto results_vec = params_future.get();
            _chessboard_square_size = results_vec[0].as_double();
            _marker_frame_name = results_vec[1].as_string();
            _rgb_frame_name = results_vec[2].as_string();
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN_STREAM(get_logger(), e.what() << ". Exiting...");
            return -1;
        }

        // rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
        // _camera_sub = create_subscription<sensor_msgs::msg::Image>(image_topic_name, qos_settings,
        //                                                            [this](const sensor_msgs::msg::Image::SharedPtr msg)
        //                                                            {
        //                                                                RCLCPP_INFO(get_logger(), "Received RGB image. Saving frame.");
        //                                                                _rgb_frame_name = msg->header.frame_id;
        //                                                                _camera_sub.reset();
        //                                                            });
                                                                        //   _rgb_frame_name = "camera_1/camera_base";

        RCLCPP_INFO_STREAM(get_logger(), "Chessboard square size: " << _chessboard_square_size << " [m].");
        RCLCPP_INFO_STREAM(get_logger(), "Marker frame name: " << _marker_frame_name);
        RCLCPP_INFO_STREAM(get_logger(), "Camera frame name: " << _rgb_frame_name);

        return 0;
    }

} // namespace probe_calibration

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(probe_calibration::ProbeCalibration)
