#include "rgb_diff/rgb_diff_server.hpp"

namespace rgb_diff_action_server
{
    RgbDiffActionServer::RgbDiffActionServer(const rclcpp::NodeOptions &options)
        : Node("rgb_diff", options)
    {
        helpers::commons::setLoggerLevelFromParameter(this);
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)); //.transient_local();

        _transforms_buffer = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::Duration(std::chrono::seconds(5)));
        _transform_listener = std::make_unique<tf2_ros::TransformListener>(*_transforms_buffer);
        // save security area masks
        this->set_security_area_masks();
        // robot_mask::RobotMask robot_mask_object(this);
        _robot_mask_object = new robot_mask::RobotMask(this);
        // SUBSCRIBERS
        _rgb_images_sub = create_subscription<custom_interfaces::msg::CamerasData>("cameras_data", qos_settings,
                                                                                   [this](const custom_interfaces::msg::CamerasData::SharedPtr _rgb_images)
                                                                                   {
                                                                                       helpers::Timer time("rgb_diff operation took: ", true);

                                                                                       RCLCPP_DEBUG(this->get_logger(),
                                                                                                    "RGB images message received");
                                                                                       _buffered_rgb_images = _rgb_images;
                                                                                       //UNCOMMENT FOR TIMING
                                                                                       // using clk = std::chrono::steady_clock;
                                                                                       // auto start = clk::now();

                                                                                       auto result = std::make_shared<RgbDiffAction::Result>();

                                                                                       if (_rgbdiff_background_cam1.empty() ||
                                                                                           _rgbdiff_background_cam2.empty())
                                                                                       {
                                                                                           this->_publisher_rgbdiff_result->publish(
                                                                                               custom_interfaces::msg::RgbDiffResult());
                                                                                           // to set background images as
                                                                                           RCLCPP_ERROR(this->get_logger(), "Not performing diff operation - background images are not set");
                                                                                           return;
                                                                                       }
                                                                                       else if (!_rgb_images ||
                                                                                                _rgb_images->header.stamp ==
                                                                                                    builtin_interfaces::msg::Time())
                                                                                       {
                                                                                           this->_publisher_rgbdiff_result->publish(
                                                                                               custom_interfaces::msg::RgbDiffResult());
                                                                                           RCLCPP_ERROR(
                                                                                               this->get_logger(),
                                                                                               "Not performing diff operation - empty header in the RGB images message or no RGB images to process");
                                                                                           return;
                                                                                       }

                                                                                       if (_last_processed_msg_timestamp ==
                                                                                           _rgb_images->header.stamp)
                                                                                           RCLCPP_WARN(get_logger(),
                                                                                                       "New message has not arrived yet. Processing old message.");
                                                                                       _last_processed_msg_timestamp = _rgb_images->header.stamp;
                                                                              
                                                                                       cv::Mat robot_mask_cam1;
                                                                                       cv::Mat robot_mask_cam2;
                                                                                       _robot_mask_object->prepareRobotMask(_last_processed_msg_timestamp, robot_mask_cam1, robot_mask_cam2);

                                                                                       helpers::converters::rosImageToCV(_rgb_images->cam1_rgb, this->cam1_rgb);
                                                                                       helpers::converters::rosImageToCV(_rgb_images->cam2_rgb, this->cam2_rgb);
                                                                         
                                                                         

                                                                                       // perform background segmentation
                                                                                       this->background_substractor(
                                                                                           this->cam1_rgb,
                                                                                           this->_rgbdiff_background_cam1,
                                                                                           this->result_cam1);

                                                                                       this->background_substractor(
                                                                                           this->cam2_rgb,
                                                                                           this->_rgbdiff_background_cam2,
                                                                                           this->result_cam2);
                                                                                    
                                                                                       
                                                                                        _robot_mask_object->waitForMasks();
                                                                                       
                                                                           
                                                                        
                                                                                       // apply security area masks
                                                                                       cv::bitwise_and(
                                                                                           this->result_cam1,
                                                                                           this->_sec_area_cam1_mask,
                                                                                           this->result_cam1);
                                                                                       cv::bitwise_and(
                                                                                           this->result_cam2,
                                                                                           this->_sec_area_cam2_mask,
                                                                                           this->result_cam2);
                                                                                
                                                                        
                                                                                       // substract any movement in the picture attributable to robot
                                                                                       cv::subtract(this->result_cam1, robot_mask_cam1, this->result_cam1);
                                                                                       cv::subtract(this->result_cam2, robot_mask_cam2, this->result_cam2);
                                                            

                                                                                       // substract any robot pixels from the background image
                                                                                       cv::subtract(this->result_cam1, this->bckgrnd_robot_mask_cam1, this->result_cam1);
                                                                                       cv::subtract(this->result_cam2, this->bckgrnd_robot_mask_cam2, this->result_cam2);
                                                                
                                                                        

                                                                                       int change_sec_area =
                                                                                           cv::countNonZero(
                                                                                               this->result_cam1) +
                                                                                           cv::countNonZero(
                                                                                               this->result_cam2);

                                                                                       //PUBLISH BOOL FOR SECURTY AREA CHANGE
                                                                                       std_msgs::msg::Bool::UniquePtr affirmative(
                                                                                           new std_msgs::msg::Bool());

                                                                                       if (change_sec_area <=
                                                                                           this->_rgbdiff_scene_change_threshold)
                                                                                       {
                                                                                           affirmative->data = false;
                                                                                           _publisher_security_area_changed->publish(
                                                                                               std::move(
                                                                                                   affirmative));
                                                                                       }
                                                                                       else
                                                                                       {
                                                                                           affirmative->data = true;
                                                                                           _publisher_security_area_changed->publish(
                                                                                               std::move(
                                                                                                   affirmative));
                                                                                       }

                                                                                       // UNCOMMENT FOR TIMING
                                                                                       // auto stop = clk::now();
                                                                                       // auto duration = std::chrono::duration<double, std::milli>(stop - start);
                                                                                       // RCLCPP_INFO(get_logger(), "[EXECUTION TIME]: " + std::to_string(duration.count()) + " ms");
                                                                                   });

        _rgbdiff_set_background_sub = create_subscription<std_msgs::msg::Bool>("rgbdiff_set_background", qos_settings,
                                                                               [this](const std_msgs::msg::Bool::SharedPtr rgbdiff_background_trigger)
                                                                               {
                                                                                   if (rgbdiff_background_trigger->data &&
                                                                                       _buffered_rgb_images !=
                                                                                           nullptr)
                                                                                   {
                                                                                       if (_buffered_rgb_images)
                                                                                       {

                                                                                           RCLCPP_INFO(
                                                                                               this->get_logger(),
                                                                                               "Setting new background images");
                                                                                           // read rgb from rgb topic and set as the new background

                                                                                           helpers::converters::rosImageToCV(
                                                                                               this->_buffered_rgb_images->cam1_rgb,
                                                                                               this->_rgbdiff_background_cam1);
                                                                                           helpers::converters::rosImageToCV(
                                                                                               this->_buffered_rgb_images->cam2_rgb,
                                                                                               this->_rgbdiff_background_cam2);
                                                                                           // std::cout<<this->_rgbdiff_background_cam2.data<<std::endl;

                                                                                           {

                                                                                               helpers::Timer time("geting masks took: ",true);
                                                                                               _robot_mask_object->prepareRobotMask(_last_processed_msg_timestamp,this->bckgrnd_robot_mask_cam1,this->bckgrnd_robot_mask_cam2);
                                                                                                void waitForMasks();

                                                                                           }
                                                                        
                                                                                       }
                                                                                       else
                                                                                       {
                                                                                           RCLCPP_ERROR(
                                                                                               this->get_logger(),
                                                                                               "No RGB images recevied from RgbImages topic to set as background");
                                                                                       }
                                                                                   }
                                                                               });

        _rgbdiff_pixel_threshold_sub = create_subscription<std_msgs::msg::Int32>("rgbdiff_pixel_threshold",
                                                                                 qos_settings,
                                                                                 [this](const std_msgs::msg::Int32::SharedPtr rgbdiff_pixel_threshold)
                                                                                 {
                                                                                     RCLCPP_DEBUG(this->get_logger(),
                                                                                                  "New RGB pixel diff threshold received");
                                                                                     _rgbdiff_pixel_threshold = rgbdiff_pixel_threshold->data;
                                                                                 });

        _rgbdiff_scene_change_threshold_sub = create_subscription<std_msgs::msg::Int32>(
            "rgbdiff_scene_change_threshold", qos_settings,
            [this](const std_msgs::msg::Int32::SharedPtr rgbdiff_scene_change_threshold)
            {
                RCLCPP_DEBUG(this->get_logger(), "New RGB scene change threshold received");
                _rgbdiff_scene_change_threshold = rgbdiff_scene_change_threshold->data;
            });

        // PUBLISHERS
        _publisher_rgbdiff_result = create_publisher<custom_interfaces::msg::RgbDiffResult>("rgbdiff_result",
                                                                                            qos_settings);
        _publisher_security_area_changed = create_publisher<std_msgs::msg::Bool>("security_trigger", qos_settings);

        this->_action_server = rclcpp_action::create_server<RgbDiffAction>(
            this,
            "rgb_diff",
            std::bind(&RgbDiffActionServer::_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RgbDiffActionServer::_handle_cancel, this, std::placeholders::_1),
            std::bind(&RgbDiffActionServer::_handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "rgb_diff initialized");

        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    void RgbDiffActionServer::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }
    void RgbDiffActionServer::shutDownNode()
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    std::string RgbDiffActionServer::_getParam(std::string param_name)
    {
        RCLCPP_INFO(this->get_logger(), "Getting ip of detectron server...");
        std::string param_string;
        try
        {
            json data = helpers::commons::getParameter(param_name);
            param_string = data.dump();
        }
        catch (const json::exception &e)
        {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Reading \"" << param_name << "\" JSON error: " << e.what());
        }
        return param_string;
    }

    rclcpp_action::GoalResponse RgbDiffActionServer::_handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                                  std::shared_ptr<const RgbDiffAction::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    RgbDiffActionServer::_handle_cancel(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void RgbDiffActionServer::_handle_accepted(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle)
    {
        (void)goal_handle;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&RgbDiffActionServer::_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    std::shared_ptr<cv::Mat>
    RgbDiffActionServer::background_substractor(cv::Mat &target, cv::Mat &background, cv::Mat &result)
    {
        cvtColor(target, target, cv::COLOR_BGR2GRAY);
        if (background.channels() != 1)
        {
            cvtColor(background, background, cv::COLOR_BGR2GRAY);
        }
        result = cv::abs(background - target);
        cv::threshold(result, result, this->_rgbdiff_pixel_threshold, 255, cv::THRESH_BINARY);
        auto ret_mask = std::make_shared<cv::Mat>(result);

        return ret_mask;
    }
    void RgbDiffActionServer::set_security_area_masks()
    {

        // UNCOMMENT WHEN MASKS READY
        this->_sec_area_cam1_mask = cv::imread("/root/ros2_ws/src/avena_ros2/rgb_diff/masks/cam1_mask.png",
                                               cv::IMREAD_GRAYSCALE);

        this->_sec_area_cam2_mask = cv::imread("/root/ros2_ws/src/avena_ros2/rgb_diff/masks/cam2_mask.png",
                                               cv::IMREAD_GRAYSCALE);
    }

    void RgbDiffActionServer::_execute(const std::shared_ptr<GoalHandleRgbDiffAction> goal_handle)
    {
        if (status != custom_interfaces::msg::Heartbeat::RUNNING)
        {
            RCLCPP_WARN(this->get_logger(), "Node isn't in RUNNING state");
            return;
        }
        auto result = std::make_shared<RgbDiffAction::Result>();
        goal_handle->succeed(result);
        return;
    }

    std::vector<std::string>
    RgbDiffActionServer::_removeRobotPrefix(std::vector<std::string> link_names, std::string robot_prefix)
    {

        std::vector<std::string> links_without_prefix(link_names.size());
        for (size_t i = 0; i < link_names.size(); i++)
            links_without_prefix[i] = link_names[i].substr((robot_prefix.length() + 1),
                                                           link_names[i].length() - robot_prefix.length() - 1);
        return links_without_prefix;
    }

} // namespace rgb_diff_action_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rgb_diff_action_server::RgbDiffActionServer)
