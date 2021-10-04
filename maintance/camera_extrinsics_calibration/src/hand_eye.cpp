#include "hand_eye.hpp"

PclCalibrator::PclCalibrator(const rclcpp::NodeOptions &options)
    : Node("hand_eye_calibration", options)
{
    helpers::commons::setLoggerLevel(get_logger(), "info");
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    _getParameter("camera_id", _camera_index, 1);
    _getParameter("samples_amount", _samples_amount, 5);

    _initializeSubscribers(qos_settings);

    // _change_tool_client = create_client<custom_interfaces::srv::ChangeTool>("change_tool");

    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    _transforms_buffer = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::Duration(std::chrono::seconds(5)));
    _transform_listener = std::make_unique<tf2_ros::TransformListener>(*_transforms_buffer);

    _static_broadcaster = std::make_unique<tf2_ros::StaticTransformBroadcaster>(get_node_topics_interface());

    // _cam1_robot_positions.resize(1);
    // _cam1_robot_positions[0] = Eigen::Translation3f(0.571, -0.902, 0.701) * Eigen::Quaternionf(0.603, -0.058, 0.337, -0.720);

    // _cam2_robot_positions.resize(1);
    // _cam2_robot_positions[0] = Eigen::Translation3f(0.422, 0.457, 0.477) * Eigen::Quaternionf(0.669, -0.022, 0.510, 0.540);

    _action_server = rclcpp_action::create_server<HandEyeAction>(
        this,
        "calibrate_cameras",
        std::bind(&PclCalibrator::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PclCalibrator::_handleCancel, this, std::placeholders::_1),
        std::bind(&PclCalibrator::_handleAccepted, this, std::placeholders::_1));
}

void PclCalibrator::_getParameter(std::string param_name, int &param_var, int default_value)
{

    rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
    parameter_descriptor.name = param_name;
    parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    parameter_descriptor.read_only = false;
    this->declare_parameter(param_name, default_value, parameter_descriptor);
    this->get_parameter<int>(param_name, param_var);
}

void PclCalibrator::_initializeSubscribers(const rclcpp::QoS &qos_settings)
{
    rclcpp::QoS qos_settings_info = rclcpp::QoS(rclcpp::KeepLast(1));
    std::string topic_prefix = _camera_prefix + std::to_string(_camera_index);
    _camera_rgb_sub = create_subscription<sensor_msgs::msg::Image>(topic_prefix + _camera_rgb_topic, qos_settings,
                                                                   [this](sensor_msgs::msg::Image::SharedPtr msg)
                                                                   {
                                                                       RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Received RGB image (message prints once every second)");
                                                                       std::lock_guard<std::mutex> lg(_rgb_image_mtx);
                                                                       _rgb_image = msg;
                                                                   });

    _camera_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(topic_prefix + _camera_info_topic, qos_settings_info,
                                                                         [this](sensor_msgs::msg::CameraInfo::SharedPtr msg)
                                                                         {
                                                                             RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000, "Received camera info (message prints once every second)");
                                                                             std::lock_guard<std::mutex> lg(_camera_info_mtx);
                                                                             _camera_info = msg;
                                                                         });
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

void PclCalibrator::_rotateSamples(std::vector<Eigen::Affine3f> &input_samples, const size_t rotations_per_sample, std::vector<Eigen::Affine3f> &output_samples)
{
    for (auto &initial_sample : input_samples)
        for (size_t i = 0; i < rotations_per_sample; i++)
            output_samples.push_back(initial_sample.rotate(helpers::vision::assignRotationMatrixAroundZ(2 * M_PI / rotations_per_sample)));
}

void PclCalibrator::_execute(const std::shared_ptr<GoalHandleHandEye> goal_handle)
{
    auto result = std::make_shared<HandEyeAction::Result>();

    if (_poses.size() >= static_cast<size_t>(_samples_amount))
        _poses.clear();

    auto calibration_return_code = _initiateCalibration();

    if (calibration_return_code == 0)
        goal_handle->succeed(result);
    else
        goal_handle->abort(result);

    if(_poses.size() > 0)
        RCLCPP_INFO(get_logger(), "Sample: " + std::to_string(_poses.size()) +"/" +std::to_string(_samples_amount));

    return;
}

int PclCalibrator::_initiateCalibration()
{
    auto single_camera_calibration = [this]() -> std::optional<Eigen::Affine3f>
    {
        // int nr_cam_fails = 0;
        // int nr_cam_succeses = 0;

        // std::vector<Eigen::Affine3f> cam_robot_positions;
        // _rotateSamples(positions, AMOUNT_SAMPLES_PER_CAMERA, cam_robot_positions);
        // std::vector<Eigen::Affine3f> cam_to_base_samples;

        // for (auto position = cam_robot_positions.begin(); position != cam_robot_positions.end(); position++)
        // {
        // RCLCPP_INFO_STREAM(get_logger(), "Processing position number: " << std::distance(cam_robot_positions.begin(), position) + 1 << " of " << cam_robot_positions.size());
        // if (_moveRobot(*position) != 0)
        // {
        //     RCLCPP_WARN_STREAM(this->get_logger(), "Robot was not able to reach position!");
        //     continue;
        // }

        Eigen::Affine3f cam_to_base_sample;
        std::this_thread::sleep_for(std::chrono::seconds(2));
        int calibrate_return_code = calibrate(cam_to_base_sample, _current_threshold);
        if (calibrate_return_code == CalibrateReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Calibration for single position successfull");
        }
        else if (calibrate_return_code == CalibrateReturnCode::CAMERA_INFO_ERROR)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Cannot obtain camera info from topic");
        }
        else if (calibrate_return_code == CalibrateReturnCode::IMAGE_ERROR)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Cannot obtain RGB image from topic");
        }
        else if (calibrate_return_code == CalibrateReturnCode::WORLD_TO_END_EFFECTOR_TF_ERROR)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Cannot obtain TF from \"world\" to \"end effector\"");
        }
        else if (calibrate_return_code == CalibrateReturnCode::CAMERA_BASE_TO_RGB_LINK_TF_ERROR)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Cannot obtain TF from \"camera base\" to \"rgb camera link\"");
        }
        else if (calibrate_return_code == CalibrateReturnCode::CALIBRATION_MAT_NOT_FOUND)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Cannot find calibration mat");
        }
        else if (calibrate_return_code == CalibrateReturnCode::PNP_ERROR)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "PnP algorith was not able to solve the problem, skipping sample...");
        }

        if (calibrate_return_code != CalibrateReturnCode::SUCCESS)
        {
            // nr_cam_fails++;
            // continue;
            RCLCPP_WARN_STREAM(this->get_logger(), "Calibration failed for current position.");
            return std::nullopt;
        }

        // nr_cam_succeses++;
        // cam_to_base_samples.push_back(cam_to_base_sample);
        // }

        // Eigen::Affine3f cam_pose(Eigen::Matrix4f::Zero());
        // RCLCPP_INFO(get_logger(), "Calculation of camera transforms from all poses");
        // if (cam_to_base_samples.size() == 0)
        //     RCLCPP_ERROR_STREAM(this->get_logger(), "Calibration was not succesful!");

        // for (auto &pose : cam_to_base_samples)
        //     cam_pose.matrix() = cam_pose.matrix() + pose.matrix();

        // std::cout << "cam fails: " << nr_cam_fails << ", cam succeses: " << nr_cam_succeses << std::endl;
        // std::cout << "cam sample size: " << cam_to_base_samples.size() << std::endl;

        // if (cam_to_base_samples.size() > 0)
        //     cam_pose.matrix() = cam_pose.matrix() / cam_to_base_samples.size();
        // std::optional<Eigen::Affine3f> cam_to_base_sample;
        return cam_to_base_sample;
    };

    RCLCPP_INFO(get_logger(), "Calibrating camera " + std::to_string(_camera_index) + " sample nr: " + std::to_string(_poses.size() + 1));

    auto cam_pose_sample = single_camera_calibration();
    RCLCPP_INFO(get_logger(), "calibration for sample nr: " + std::to_string(_poses.size() + 1) + " finished");
    if (cam_pose_sample != std::nullopt)
    {
        _poses.push_back(*cam_pose_sample);
        RCLCPP_INFO(get_logger(), "status: Success");
    }
    else{
        RCLCPP_INFO(get_logger(), "status: Fail");
        return 1;
    }

    if (_poses.size() >= static_cast<size_t>(_samples_amount))
    {

        Eigen::Affine3f cam_pose(Eigen::Matrix4f::Zero());
        for (auto &pose : _poses)
            cam_pose.matrix() = cam_pose.matrix() + pose.matrix();
        cam_pose.matrix() = cam_pose.matrix() / _poses.size();
        RCLCPP_INFO(get_logger(), "Found " + std::to_string(_poses.size()) + " valid poses for camera " + std::to_string(_camera_index));

        if (cam_pose.matrix() != Eigen::Matrix4f::Zero())
        {
            std::string base = _camera_prefix + std::to_string(_camera_index) + _camera_base;
            //     _displayTransform(cam_pose, WORLD, base);
            std::string filename = _camera_prefix + std::to_string(_camera_index) + "_calibration.yaml";
            _saveToYaml(cam_pose, WORLD, base, filename);
        }
        _poses.clear();
    }

    return 0;
}

int PclCalibrator::_callSimpleAction(std::string name, std::chrono::seconds timeout)
{
    auto action_client = rclcpp_action::create_client<custom_interfaces::action::SimpleAction>(this, name);
    // RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " + name + "action server...");
    if (waitForServer<custom_interfaces::action::SimpleAction>(action_client))
        return 1;
    // RCLCPP_INFO_STREAM(this->get_logger(), "Send goal to \"" << name << "\" action.");
    auto goal_msg = custom_interfaces::action::SimpleAction::Goal();
    // send empty goal
    auto goal_future = action_client->async_send_goal(goal_msg);
    // wait for results
    if (goal_future.wait_for(timeout) == std::future_status::timeout)
    {
        // RCLCPP_ERROR(this->get_logger(), "Timeout while waiting for " + name + " results...");
        return 1;
    }
    auto result_future = action_client->async_get_result(goal_future.get());
    return _validateResponse(result_future, name);
}

int PclCalibrator::_validateResponse(ActionResult &result, std::string &action_name)
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
        rclcpp::Duration duration(std::chrono::milliseconds(1000));
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

// void PclCalibrator::_publishRobotToTargerTF()
// {

//     Eigen::Matrix4f pattern_robot;
//     pattern_robot << 0.707, 0.707, 0, -BOARD_BOX_SIZE,
//         -0.707, 0.707, 0, -BOARD_BOX_SIZE * 1.5,
//         0, 0, 1, -0.01,
//         0, 0, 0, 1;

//     Eigen::Affine3f cal_ee(pattern_robot);
//     Eigen::Matrix3f cv_to_eigen_rot;

//     cv_to_eigen_rot.col(0) = -1 * Eigen::Vector3f::UnitY();
//     cv_to_eigen_rot.col(1) = -1 * Eigen::Vector3f::UnitX();
//     cv_to_eigen_rot.col(2) = cv_to_eigen_rot.col(0).cross(cv_to_eigen_rot.col(1));

//     Eigen::Affine3f cv_to_eigen_aff = Eigen::Translation3f(0.0, 0.0, 0.0) * cv_to_eigen_rot;

//     _publishTransform(cal_ee.inverse(), PANDA_GRIPPER_LINK, "cv_to_eigen", true);
//     _publishTransform(cv_to_eigen_aff, "cv_to_eigen", PANDA_EE_LINK, true);
// }

CalibrateReturnCode PclCalibrator::calibrate(Eigen::Affine3f &out_transform, int &in_out_curr_threshold)
{
    auto find_tf = [this](const sensor_msgs::msg::Image::SharedPtr &rgb_img, const cv::Mat &view, const cv::Mat &equ_img, const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs, const int threshold, Eigen::Affine3f &out_transform) -> CalibrateReturnCode
    {
        std::vector<cv::Point2f> corners1;
        cv::Mat rotation_vector; // Rotation in axis-angle form
        cv::Mat translation_vector;

        std::vector<cv::Point3f> objectPoints;
        cv::Size patternSize(CHESS_BOARD_PATTERN_WIDTH, CHESS_BOARD_PATTERN_HIGHT);
        for (int j = 0; j < patternSize.height; j++)
            for (int i = 0; i < patternSize.width; i++)
                objectPoints.push_back(cv::Point3f(i * BOARD_BOX_SIZE, j * BOARD_BOX_SIZE, 0));

        RCLCPP_DEBUG_STREAM(get_logger(), "Checking threshold: " << threshold);
        cv::Mat chessboard_mask;
        cv::threshold(equ_img, chessboard_mask, threshold, 255, cv::THRESH_BINARY);

        RCLCPP_DEBUG(get_logger(), "Finding chessboard corners...");
        bool found = false;
        if (view.empty() != 1)
            found = cv::findChessboardCorners(chessboard_mask, patternSize, corners1, cv::CALIB_CB_FAST_CHECK); //This will detect pattern
        RCLCPP_DEBUG(get_logger(), "...done");

        if (found)
        {
            if (_display_cv)
            {
                cv::Mat resized_mask;
                cv::resize(chessboard_mask, resized_mask, chessboard_mask.size() / 4);
                cv::imshow("chessboard_mask", resized_mask);
                cv::waitKey(0);
            }

            cv::Mat gray_img;
            cv::cvtColor(view, gray_img, CV_BGR2GRAY);
            cv::cornerSubPix(gray_img, corners1, cv::Size(11, 11), cv::Size(3, 3), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
            cv::solvePnP(objectPoints, corners1, camera_matrix, dist_coeffs, rotation_vector, translation_vector);
            cv::Mat aa_cv;
            cv::Rodrigues(rotation_vector, aa_cv);
            Eigen::Matrix3f rot_matrix;
            cv::cv2eigen(aa_cv, rot_matrix);

            Eigen::Vector3f vec;
            cv::cv2eigen(translation_vector, vec);

            out_transform = Eigen::Translation3f(vec) * Eigen::Quaternionf(rot_matrix);

            Eigen::Affine3f world_to_end_effector_tf;
            Eigen::Affine3f rgb_to_base;

            std::string camera_base = _camera_prefix + std::to_string(_camera_index) + _camera_base;

            if (_lookupTransform(WORLD, PANDA_EE_LINK, now(), world_to_end_effector_tf) == 1)
                return CalibrateReturnCode::WORLD_TO_END_EFFECTOR_TF_ERROR;

            if (_lookupTransform(rgb_img->header.frame_id, camera_base, now(), rgb_to_base) == 1)
                return CalibrateReturnCode::CAMERA_BASE_TO_RGB_LINK_TF_ERROR;

            std::vector<cv::Point3d> point3D;
            std::vector<cv::Point2d> point2D;
            point3D.push_back(cv::Point3d(0, 0, -0.1));
            point3D.push_back(cv::Point3d(0.1, 0, 0));
            point3D.push_back(cv::Point3d(0, 0.1, 0));

            cv::projectPoints(point3D, rotation_vector, translation_vector, camera_matrix, dist_coeffs, point2D);

            float x1, x2, y1, y2;
            x1 = corners1[3].x - corners1[0].x;
            x2 = point2D[1].x - corners1[0].x;
            y1 = corners1[3].y - corners1[0].y;
            y2 = point2D[1].y - corners1[0].y;
            float angle_x = acos((x1 * x2 + y1 * y2) / (sqrt(x1 * x1 + y1 * y1) * sqrt(x2 * x2 + y2 * y2))) * 180 / M_PI;

            float x_1, x_2, y_1, y_2;
            x_1 = corners1[8].x - corners1[0].x;
            x_2 = point2D[2].x - corners1[0].x;
            y_1 = corners1[8].y - corners1[0].y;
            y_2 = point2D[2].y - corners1[0].y;
            float angle_y = acos((x_1 * x_2 + y_1 * y_2) / (sqrt(x_1 * x_1 + y_1 * y_1) * sqrt(x_2 * x_2 + y_2 * y_2))) * 180 / M_PI;

            if (angle_x > 0.3 || std::isnan(angle_x) || angle_y > 0.3 || std::isnan(angle_y))
            {
                return CalibrateReturnCode::PNP_ERROR;
            }

            out_transform = (out_transform.inverse() * rgb_to_base).inverse();
            out_transform = world_to_end_effector_tf * out_transform.inverse();

            if (_display_cv)
            {

                //Tp drow x,y z axis on image.
                cv::line(view, corners1[0], point2D[0], cv::Scalar(255, 0, 0), 3); //z
                cv::line(view, corners1[0], point2D[1], cv::Scalar(0, 0, 255), 3); //x
                cv::line(view, corners1[0], point2D[2], cv::Scalar(0, 255, 0), 3); //y
                cv::putText(view, "x", cv::Point(point2D[1].x - 10, point2D[1].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 255), 2);
                cv::putText(view, "y", cv::Point(point2D[2].x - 10, point2D[2].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 0), 2);
                cv::putText(view, "z", cv::Point(point2D[0].x - 10, point2D[0].y - 10), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255, 0, 0), 2);
                cv::circle(view, point2D[0], 3, cv::Scalar(255, 0, 0), 4, 8, 0);
                cv::circle(view, point2D[1], 3, cv::Scalar(0, 0, 255), 4, 8, 0);
                cv::circle(view, point2D[2], 3, cv::Scalar(0, 255, 0), 4, 8, 0);

                // Display image.
                cv::Mat resized_img;
                cv::drawChessboardCorners(view, patternSize, corners1, found);
                cv::resize(view, resized_img, view.size() / 4);
                cv::namedWindow(rgb_img->header.frame_id, cv::WINDOW_NORMAL);
                cv::imshow(rgb_img->header.frame_id, resized_img);
                cv::waitKey(0);
            }

            return CalibrateReturnCode::SUCCESS;
        }
        return CalibrateReturnCode::CALIBRATION_MAT_NOT_FOUND;
    };

    auto [image, cam_info] = _getData();

    if (!image)
        return CalibrateReturnCode::IMAGE_ERROR;

    if (!cam_info)
        return CalibrateReturnCode::CAMERA_INFO_ERROR;

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << cam_info->k[0], cam_info->k[1], cam_info->k[2], cam_info->k[3], cam_info->k[4], cam_info->k[5], cam_info->k[6], cam_info->k[7], cam_info->k[8]);
    cv::Mat dist_coeffs = (cv::Mat_<double>(8, 1) << cam_info->d[0], cam_info->d[1], cam_info->d[2], cam_info->d[3], cam_info->d[4], cam_info->d[5], cam_info->d[6], cam_info->d[7]);

    cv::Mat view;
    cv::Mat gray_img;

    RCLCPP_INFO(get_logger(), "Preprocessing of RGB image");
    helpers::converters::rosImageToCV(*image, view);
    cv::cvtColor(view, gray_img, CV_BGR2GRAY);

    cv::Mat equ_img;
    cv::equalizeHist(gray_img, equ_img);

    if (in_out_curr_threshold == -1)
    {
        for (int threshold = 0; threshold <= 255; threshold++)
        {
            if (find_tf(image, view, equ_img, camera_matrix, dist_coeffs, threshold, out_transform) == CalibrateReturnCode::SUCCESS)
            {
                in_out_curr_threshold = threshold;
                return CalibrateReturnCode::SUCCESS;
            }
        }
        return CalibrateReturnCode::CALIBRATION_MAT_NOT_FOUND;
    }
    else
    {
        // First check whether current threshold is valid
        if (find_tf(image, view, equ_img, camera_matrix, dist_coeffs, in_out_curr_threshold, out_transform) == CalibrateReturnCode::SUCCESS)
        {
            return CalibrateReturnCode::SUCCESS;
        }

        int low_bound = in_out_curr_threshold - 1;
        int high_bound = in_out_curr_threshold + 1;
        while (true)
        {
            // Exit condition
            if (low_bound < MIN_PIXEL_VAL && high_bound > MAX_PIXEL_VAL)
            {
                in_out_curr_threshold = -1;
                break;
            }

            // Check threshold lower than current threshold
            if (low_bound >= MIN_PIXEL_VAL)
            {
                if (find_tf(image, view, equ_img, camera_matrix, dist_coeffs, low_bound, out_transform) == CalibrateReturnCode::SUCCESS)
                {
                    in_out_curr_threshold = low_bound;
                    return CalibrateReturnCode::SUCCESS;
                }
            }

            // Check threshold higher than current threshold
            if (high_bound <= MAX_PIXEL_VAL)
            {
                if (find_tf(image, view, equ_img, camera_matrix, dist_coeffs, high_bound, out_transform) == CalibrateReturnCode::SUCCESS)
                {
                    in_out_curr_threshold = high_bound;
                    return CalibrateReturnCode::SUCCESS;
                }
            }

            low_bound--;
            high_bound++;
        }
    }

    return CalibrateReturnCode::CALIBRATION_MAT_NOT_FOUND;
}

std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr> PclCalibrator::_getData()
{
    // Get RGB data
    sensor_msgs::msg::Image::SharedPtr image;

    std::lock_guard<std::mutex> lg_rgb(_rgb_image_mtx);
    image = std::make_shared<sensor_msgs::msg::Image>(*_rgb_image);

    // Camera info
    sensor_msgs::msg::CameraInfo::SharedPtr cam_info;

    std::lock_guard<std::mutex> lg_info(_camera_info_mtx);
    cam_info = std::make_shared<sensor_msgs::msg::CameraInfo>(*_camera_info);

    return {image, cam_info};
}

void PclCalibrator::_saveToYaml(Eigen::Affine3f &camera_transform, std::string parent, std::string child, std::string filename)
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
    RCLCPP_INFO(get_logger(), "Transform saved in " + filename);

}

// void PclCalibrator::_publishTransform(const Eigen::Affine3f &in_transform, const std::string &parent,
//                                      const std::string &child, bool is_static)
// {

//     geometry_msgs::msg::TransformStamped transformStamped;
//     if (!is_static)
//         transformStamped.header.stamp = now();
//     transformStamped.header.frame_id = parent;
//     transformStamped.child_frame_id = child;
//     transformStamped.transform.translation.x = in_transform.translation().x();
//     transformStamped.transform.translation.y = in_transform.translation().y();
//     transformStamped.transform.translation.z = in_transform.translation().z();
//     Eigen::Quaternionf quat(in_transform.rotation());
//     transformStamped.transform.rotation.x = quat.x();
//     transformStamped.transform.rotation.y = quat.y();
//     transformStamped.transform.rotation.z = quat.z();
//     transformStamped.transform.rotation.w = quat.w();
//     if (is_static)
//         _static_broadcaster->sendTransform({transformStamped});
//     else
//         _tf_broadcaster->sendTransform(transformStamped);
// }

void PclCalibrator::_displayTransform(const Eigen::Affine3f &in_transform, const std::string &parent,
                                      const std::string &child)
{
    Eigen::Quaternionf rotation(in_transform.rotation());
    std::cout << "ros2 run tf2_ros static_transform_publisher " << in_transform.translation().x() << " " << in_transform.translation().y() << " " << in_transform.translation().z() << " " << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << " " << parent << " " << child << std::endl;
    // _publishTransform(in_transform, parent, child);
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PclCalibrator)