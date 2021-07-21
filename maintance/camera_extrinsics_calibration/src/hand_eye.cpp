#include <hand_eye.hpp>
using namespace cv;
PclCalibrator::PclCalibrator(const rclcpp::NodeOptions &options)
    : Node("calibrate", options)
{

    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1));
    _subscriptions_manager = std::make_shared<helpers::SubscriptionsManager>(get_node_topics_interface());

    _subscriptions_manager->createSubscription(_camera1_rgb_topic, "sensor_msgs/Image", qos_settings);
    _subscriptions_manager->createSubscription(_camera2_rgb_topic, "sensor_msgs/Image", qos_settings);
    _subscriptions_manager->createSubscription(_camera1_info_topic, "sensor_msgs/CameraInfo", qos_settings);
    _subscriptions_manager->createSubscription(_camera2_info_topic, "sensor_msgs/CameraInfo", qos_settings);

    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    _transforms_buffer = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::Duration(std::chrono::seconds(5)));
    _transform_listener = std::make_unique<tf2_ros::TransformListener>(*_transforms_buffer);

    _static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(get_node_topics_interface());

    _cam1_robot_positions.resize(1);
    _cam1_robot_positions[0] = Eigen::Translation3f(0.645, -0.617, 0.765) * Eigen::Quaternionf(0.655, -0.507, 0.243, -0.505);  


    _cam2_robot_positions.resize(1);
    _cam2_robot_positions[0] = Eigen::Translation3f(0.532, 0.256, 0.475) * Eigen::Quaternionf(0.419, 0.556, 0.345, 0.629);


    this->_action_server = rclcpp_action::create_server<HandEyeAction>(
        this,
        "hand_eye",
        std::bind(&PclCalibrator::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PclCalibrator::_handleCancel, this, std::placeholders::_1),
        std::bind(&PclCalibrator::_handleAccepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse PclCalibrator::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const HandEyeAction::Goal> /*goal*/)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PclCalibrator::_handleCancel(const std::shared_ptr<GoalHandleHandEye> /*goal_handle*/)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PclCalibrator::_handleAccepted(const std::shared_ptr<GoalHandleHandEye> goal_handle)
{
    std::thread{std::bind(&PclCalibrator::_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void PclCalibrator::_rotate_samples(std::vector<Eigen::Affine3f> &input_samples, std::vector<Eigen::Affine3f> &output_samples)
{

    int rotations_per_sample = 10;
    for (auto &initial_sample : input_samples)
        for (int i = 0; i < rotations_per_sample; i++)
            output_samples.push_back(initial_sample.rotate(helpers::vision::assignRotationMatrixAroundX(2 * M_PI / rotations_per_sample)));

    // helpers::visualization::visualize({},output_samples);
}

void PclCalibrator::_execute(const std::shared_ptr<GoalHandleHandEye> goal_handle)
{
    auto result = std::make_shared<HandEyeAction::Result>();

    int camera_index = static_cast<int>(goal_handle.get()->get_goal()->camera_index);

    if(_initiate_calibration() == 0)
        goal_handle->succeed(result);
    else
        goal_handle->abort(result);
    return;
    // std::vector<Eigen::Affine3f> cam1_robot_positions;
    // std::vector<Eigen::Affine3f> cam2_robot_positions;

    // _rotate_samples(_cam1_robot_positions, cam1_robot_positions);
    // _rotate_samples(_cam2_robot_positions, cam2_robot_positions);
    // std::vector<Eigen::Affine3f> cam1_to_base_samples;
    // std::vector<Eigen::Affine3f> cam2_to_base_samples;
    // cam1_to_base_samples.reserve(cam1_robot_positions.size());
    // cam2_to_base_samples.reserve(cam2_robot_positions.size());

    Eigen::Affine3f cam_to_base_sample;
    if (camera_index == 1)
    {
        if (calibrate(_camera1_rgb_topic, _camera1_info_topic, cam_to_base_sample) == 0){

            _cam1_to_base_samples.push_back(cam_to_base_sample);
            _displayTransform(cam_to_base_sample, PANDA_BASE_LINK, "camera_1/rgb_camera_link");

        }else{
            RCLCPP_WARN_STREAM(this->get_logger(), "Calibration for this position was not succesful!");
            goal_handle->abort(result);
            return;
        }
    }
    else if (camera_index == 2)
    {
        if (calibrate(_camera2_rgb_topic, _camera2_info_topic, cam_to_base_sample) == 0){

            _cam2_to_base_samples.push_back(cam_to_base_sample);
            _displayTransform(cam_to_base_sample, PANDA_BASE_LINK, "camera_2/rgb_camera_link");
        }else{
            RCLCPP_WARN_STREAM(this->get_logger(), "Calibration for this position was not succesful!");
            goal_handle->abort(result);
            return;
        }
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Passed camera index is not valid. ");
        goal_handle->abort(result);
        return;
    }


    if(_cam1_to_base_samples.size() < AMOUNT_SAMPLES_PER_CAMERA || _cam2_to_base_samples.size() < AMOUNT_SAMPLES_PER_CAMERA){
        RCLCPP_INFO_STREAM(this->get_logger(), "curent amount of samples for camera 1 is: " + std::to_string(_cam1_to_base_samples.size()));
        RCLCPP_INFO_STREAM(this->get_logger(), "curent amount of samples for camera 2 is: " + std::to_string(_cam2_to_base_samples.size()));
    }else{

        Eigen::Affine3f cam1_pose(Eigen::Matrix4f::Zero());
        Eigen::Affine3f cam2_pose(Eigen::Matrix4f::Zero());

        for (auto &pose : _cam1_to_base_samples)
            cam1_pose.matrix() += pose.matrix();
        for (auto &pose : _cam2_to_base_samples)
            cam2_pose.matrix() += pose.matrix();

        cam1_pose.matrix() = cam1_pose.matrix() / _cam1_to_base_samples.size();
        cam2_pose.matrix() = cam2_pose.matrix() / _cam2_to_base_samples.size();

        // publishTransform(cam1_pose, "world", "camera_1/rgb_camera_link");
        // publishTransform(cam2_pose,"world", "camera_2/rgb_camera_link");
        _displayTransform(cam1_pose, "world", "camera_1/rgb_camera_link");
        _displayTransform(cam2_pose,"world", "camera_2/rgb_camera_link");
    }

        goal_handle->succeed(result);










    
    //  Eigen::Affine3f out_transform;

    //     if (_lookupTransform("world", PANDA_GRIPPER_LINK,now(), out_transform) == 0)
    //         _displayTransform(out_transform,"world", "testowy");

    //     goal_handle->succeed(result);

    // //  without robot
    //    Eigen::Affine3f cam1_to_base_sample;
    //    int state =  calibrate(_camera1_rgb_topic, _camera1_info_topic, cam1_to_base_sample);
    //     std::cout << state << std::endl;

    //     _displayTransform(cam1_to_base_sample,"world", "camera_1/rgb_camera_link");

    //     goal_handle->succeed(result);

    return;
}

int PclCalibrator::_initiate_calibration()
{
    std::vector<Eigen::Affine3f> cam1_robot_positions;
    std::vector<Eigen::Affine3f> cam2_robot_positions;

    _rotate_samples(_cam1_robot_positions, cam1_robot_positions);
    _rotate_samples(_cam2_robot_positions, cam2_robot_positions);
    std::vector<Eigen::Affine3f> cam1_to_base_samples;
    std::vector<Eigen::Affine3f> cam2_to_base_samples;
    cam1_to_base_samples.reserve(cam1_robot_positions.size());
    cam2_to_base_samples.reserve(cam2_robot_positions.size());

    for (auto &position : cam1_robot_positions)
    {
        if (_move_robot(position) != 0)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Robot was not able to reach position!");
            continue;
        }
        Eigen::Affine3f cam1_to_base_sample;
        if (calibrate(_camera1_rgb_topic, _camera1_info_topic, cam1_to_base_sample) != 0){
            RCLCPP_WARN_STREAM(this->get_logger(), "cant find calibration board");
            continue;
        }
        cam1_to_base_samples.push_back(cam1_to_base_sample);
    }

    for (auto &position : cam2_robot_positions)
    {
        if (_move_robot(position) != 0)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Robot was not able to reach position!");
            continue;
        }
        Eigen::Affine3f cam2_to_base_sample;
        if (calibrate(_camera2_rgb_topic, _camera2_info_topic, cam2_to_base_sample) != 0){
            RCLCPP_WARN_STREAM(this->get_logger(), "cant find calibration board");
            continue;
        }
        cam2_to_base_samples.push_back(cam2_to_base_sample);
    }
    cam1_to_base_samples.shrink_to_fit();
    cam2_to_base_samples.shrink_to_fit();

    if (cam1_to_base_samples.size() == 0) //|| cam2_to_base_samples.size() == 0)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Calibration was not succesful!");
        return 1;
    }

    Eigen::Affine3f cam1_pose(Eigen::Matrix4f::Zero());
    Eigen::Affine3f cam2_pose(Eigen::Matrix4f::Zero());

    for (auto &pose : cam1_to_base_samples)
        cam1_pose.matrix() += pose.matrix();
    for (auto &pose : cam2_to_base_samples)
        cam2_pose.matrix() += pose.matrix();

    cam1_pose.matrix() = cam1_pose.matrix() / cam1_to_base_samples.size();
    cam2_pose.matrix() = cam2_pose.matrix() / cam2_to_base_samples.size();

    publishTransform(cam1_pose, "world", " camera_1/rgb_camera_link");
    publishTransform(cam2_pose,"world", "camera_2/rgb_camera_link");

    return 0;
}

int PclCalibrator::_move_robot(Eigen::Affine3f &pose)
{
    //TODO connect this with response form generate path point
    trajectory_msgs::msg::JointTrajectory path;

    //TODO connect generate path
    if (_call_generate_path(_generate_path_name, 10s, pose) != 0)
        return 1;
    if (_call_simple_action(_generate_trajectory_name, 10s) != 0)
        return 1;
    if (_call_simple_action(_execute_move_name, 30s) != 0)
        return 1;

    return 0;
}

int PclCalibrator::_call_generate_path(std::string name, std::chrono::seconds timeout, Eigen::Affine3f &pose)
{

    auto action_client = rclcpp_action::create_client<custom_interfaces::action::GeneratePathPose>(this, name);
    RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " + name + "action server...");
    if (waitForServer<custom_interfaces::action::GeneratePathPose>(action_client))
        return 1;
    RCLCPP_INFO_STREAM(this->get_logger(), "Send goal to \"" << name << "\" action.");
    auto goal_msg = custom_interfaces::action::GeneratePathPose::Goal();
    // send  goal
    helpers::converters::eigenAffineToGeometry(pose, goal_msg.end_effector_pose);
    auto goal_future = action_client->async_send_goal(goal_msg);
    // wait for results
    if (goal_future.wait_for(timeout) == std::future_status::timeout)
    {
        RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for " + name + " results...");
        return 1;
    }
    auto result_future = action_client->async_get_result(goal_future.get());

    if (result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED)
        return 0;
    else
        return 1;
}

int PclCalibrator::_call_simple_action(std::string name, std::chrono::seconds timeout)
{

    auto action_client = rclcpp_action::create_client<custom_interfaces::action::SimpleAction>(this, name);
    RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " + name + "action server...");
    if (waitForServer<custom_interfaces::action::SimpleAction>(action_client))
        return 1;
    RCLCPP_INFO_STREAM(this->get_logger(), "Send goal to \"" << name << "\" action.");
    auto goal_msg = custom_interfaces::action::SimpleAction::Goal();
    // send empty goal
    auto goal_future = action_client->async_send_goal(goal_msg);
    // wait for results
    if (goal_future.wait_for(timeout) == std::future_status::timeout)
    {
        RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for " + name + " results...");
        return 1;
    }
    auto result_future = action_client->async_get_result(goal_future.get());
    return _validate_response(result_future, name);
}

int PclCalibrator::_validate_response(action_resault &result, std::string &action_name)
{
    if (result.get().code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), "Get " + action_name + " result SUCCEEDED");
        return 0;
    }
    else if (result.get().code == rclcpp_action::ResultCode::ABORTED)
    {
        RCLCPP_ERROR(this->get_logger(), "Get " + action_name + " result ABORTED");
        return 2;
    }
    else if (result.get().code == rclcpp_action::ResultCode::CANCELED)
    {
        RCLCPP_ERROR(this->get_logger(), "Get " + action_name + " result CANCELED");
        return 1;
    }

    RCLCPP_ERROR(this->get_logger(), "Get " + action_name + " result UNKNOWN");
    return 1;
}

int PclCalibrator::_lookupTransform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &timestamp, Eigen::Affine3f &out_transform)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
        rclcpp::Duration duration(std::chrono::milliseconds(100));
        transform_stamped = _transforms_buffer->lookupTransform(target_frame, source_frame, timestamp, duration);
        helpers::converters::geometryToEigenAffine(transform_stamped.transform, out_transform);
    }
    catch (const tf2::TransformException &err)
    {
        std::cout << err.what() << std::endl;
        RCLCPP_INFO(get_logger(), err.what());
        return 1;
    }
    return 0;
}

void PclCalibrator::_publishRobotToTargerTF()
{

    Eigen::Matrix4f pattern_robot;
    pattern_robot << 0.707, 0.707, 0, -BOARD_BOX_SIZE,
        -0.707, 0.707, 0, -BOARD_BOX_SIZE * 1.5,
        0, 0, 1, -0.01,
        0, 0, 0, 1;

    Eigen::Affine3f cal_ee(pattern_robot);
    Eigen::Matrix3f cv_to_eigen_rot;

    cv_to_eigen_rot.col(0) = -1 * Eigen::Vector3f::UnitY();
    cv_to_eigen_rot.col(1) = -1 * Eigen::Vector3f::UnitX();
    cv_to_eigen_rot.col(2) = cv_to_eigen_rot.col(0).cross(cv_to_eigen_rot.col(1));

    Eigen::Affine3f cv_to_eigen_aff = Eigen::Translation3f(0.0, 0.0, 0.0) * cv_to_eigen_rot;

    publishTransform(cal_ee.inverse(), PANDA_GRIPPER_LINK, "cv_to_eigen", true);
    publishTransform(cv_to_eigen_aff, "cv_to_eigen", PANDA_EE_LINK, true);
}

int PclCalibrator::calibrate(std::string &camera_rgb_topic, std::string &cam_info_topic, Eigen::Affine3f &transform)
{

    sensor_msgs::msg::Image::SharedPtr image = _subscriptions_manager->getData<sensor_msgs::msg::Image>(camera_rgb_topic);
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info = _subscriptions_manager->getData<sensor_msgs::msg::CameraInfo>(cam_info_topic);

    if (!image || !cam_info)
        return 1;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << cam_info->k[0], cam_info->k[1], cam_info->k[2], cam_info->k[3], cam_info->k[4], cam_info->k[5], cam_info->k[6], cam_info->k[7], cam_info->k[8]);
    cv::Mat dist_coeffs = (cv::Mat_<double>(8, 1) << cam_info->d[0], cam_info->d[1], cam_info->d[2], cam_info->d[3], cam_info->d[4], cam_info->d[5], cam_info->d[6], cam_info->d[7]);

    std::vector<cv::Point2f> corners1;
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;
    cv::Mat Img;

    std::vector<cv::Point3f> objectPoints;

    bool found = false;
    std::vector<cv::Point3d> point3D;
    std::vector<cv::Point2d> point2D;

    cv::Size patternSize(CHESS_BOARD_PATTERN_WIDTH, CHESS_BOARD_PATTERN_HIGHT);
    for (int j = 0; j < patternSize.height; j++)
        for (int i = 0; i < patternSize.width; i++)
            objectPoints.push_back(cv::Point3f(i * BOARD_BOX_SIZE, j * BOARD_BOX_SIZE, 0));

    point3D.push_back(cv::Point3d(0, 0, -0.1));
    point3D.push_back(cv::Point3d(0.1, 0, 0));
    point3D.push_back(cv::Point3d(0, 0.1, 0));

    cv::Mat view;

    helpers::converters::rosImageToCV(*image, view);
    cv::cvtColor(view, Img, CV_BGR2GRAY);
    if (view.empty() != 1)
        found = cv::findChessboardCorners(Img, patternSize, corners1, cv::CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FILTER_QUADS + CALIB_CB_FAST_CHECK); //This will detect pattern

    if (found)
    {
        cv::cvtColor(view, Img, CV_BGR2GRAY);
        cornerSubPix(Img, corners1, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        cv::solvePnP(objectPoints, corners1, camera_matrix, dist_coeffs, rotation_vector, translation_vector);

        cv::Mat aa_cv;
        cv::Rodrigues(rotation_vector, aa_cv);
        Eigen::Matrix3f rot_matrix;
        cv::cv2eigen(aa_cv, rot_matrix);

        Eigen::Vector3f vec;
        cv::cv2eigen(translation_vector, vec);

        transform = Eigen::Translation3f(vec) * Eigen::Quaternionf(rot_matrix);

        const rclcpp::Time timestamp = image->header.stamp;

        Eigen::Affine3f out_transform;

        if (_lookupTransform(PANDA_BASE_LINK, PANDA_EE_LINK, now(), out_transform) == 1)
        {
            std::cout << "cant find proper tf from robot base to end effector" << std::endl;
            return 1;
        }
        transform = out_transform * transform.inverse();
        // transform = transform.inverse();


        if (_display_cv)
        {
            cv::projectPoints(point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, point2D);
            //Tp drow x,y z axis on image.
            cv::line(view, corners1[0], point2D[0], cv::Scalar(255, 0, 0), 3); //z
            cv::line(view, corners1[0], point2D[1], cv::Scalar(0, 0, 255), 3); //x
            cv::line(view, corners1[0], point2D[2], cv::Scalar(0, 255, 0), 3); //y
            cv::putText(view, "x", Point(point2D[1].x - 10, point2D[1].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255), 2);
            cv::putText(view, "y", Point(point2D[2].x - 10, point2D[2].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(view, "z", Point(point2D[0].x - 10, point2D[0].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 0, 0), 2);
            cv::circle(view, point2D[0], 3, cv::Scalar(255, 0, 0), 4, 8, 0);
            cv::circle(view, point2D[1], 3, cv::Scalar(0, 0, 255), 4, 8, 0);
            cv::circle(view, point2D[2], 3, cv::Scalar(0, 255, 0), 4, 8, 0);
            // Display image.

            cv::namedWindow(image->header.frame_id, cv::WINDOW_NORMAL);
            cv::imshow(image->header.frame_id, view);
            cv::waitKey(0);
        }
    }
    else
    {
        std::cout << "cant find calibration board in scene" << std::endl;
        return 2;
    }

    return 0;
}

void PclCalibrator::saveToYaml(Eigen::Affine3f &camera_transform, std::string parent, std::string child, std::string filename)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_extrinsics_calibration");
    package_share_directory += "/config/";
    boost::filesystem::create_directories(package_share_directory);
    auto file = boost::filesystem::ofstream(package_share_directory + filename);
    auto config = YAML::LoadFile(package_share_directory + filename);
    config["position"].push_back(camera_transform.translation().x());
    config["position"].push_back(camera_transform.translation().y());
    config["position"].push_back(camera_transform.translation().z());
    Eigen::Quaternionf rotation(camera_transform.rotation());
    config["orientation"].push_back(rotation.x());
    config["orientation"].push_back(rotation.y());
    config["orientation"].push_back(rotation.z());
    config["orientation"].push_back(rotation.w());
    config["parent"] = parent;
    config["child"] = child;
    file << config;
    file.close();
}

void PclCalibrator::publishTransform(const Eigen::Affine3f &in_transform, const std::string &parent,
                                     const std::string &child, bool is_static)
{

    geometry_msgs::msg::TransformStamped transformStamped;
    if (!is_static)
        transformStamped.header.stamp = now();
    transformStamped.header.frame_id = parent;
    transformStamped.child_frame_id = child;
    transformStamped.transform.translation.x = in_transform.translation().x();
    transformStamped.transform.translation.y = in_transform.translation().y();
    transformStamped.transform.translation.z = in_transform.translation().z();
    Eigen::Quaternionf quat(in_transform.rotation());
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    if (is_static)
        _static_broadcaster->sendTransform({transformStamped});
    else
        _tf_broadcaster->sendTransform(transformStamped);
}

void PclCalibrator::_displayTransform(const Eigen::Affine3f &in_transform, const std::string &parent,
                                      const std::string &child)
{
    Eigen::Quaternionf rotation(in_transform.rotation());
    std::cout << "ros2 run tf2_ros static_transform_publisher " << in_transform.translation().x() << " " << in_transform.translation().y() << " " << in_transform.translation().z() << " " << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << " " << parent << " " << child << std::endl;
    publishTransform(in_transform, parent, child);
}

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PclCalibrator>());

//     // PclCalibrator calibrator;

//     // calibrator.calibrate();

//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PclCalibrator)