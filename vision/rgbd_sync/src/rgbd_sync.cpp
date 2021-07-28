#include "rgbd_sync/rgbd_sync.hpp"


namespace rgbd_sync
{
    RgbdSynchronizer::RgbdSynchronizer(const rclcpp::NodeOptions &options)
        : Node("rgbd_sync", options)
    {   

        helpers::commons::setLoggerLevelFromParameter(this);
        _cant_touch_this = false;
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1));

        _rgb_images_sub = create_subscription<RgbImages>("rgb_images", qos_settings,
                                                                                       [this](const RgbImages::SharedPtr rgb_image_msg) {
                                                                                           if(_cant_touch_this)
                                                                                             return;
                                                                                           _rgb_images_data = rgb_image_msg;
                                                                                       });        
                                                                                       
        _depth_images_sub = create_subscription<DepthImages>("depth_images", qos_settings,
                                                                                       [this](const DepthImages::SharedPtr depth_image_msg) {
                                                                                           if(_cant_touch_this)
                                                                                             return;
                                                                                           _depth_images_data = depth_image_msg;
                                                                                       });


        qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)); //.transient_local();
        _rgbd_sync_publisher = create_publisher<RgbdSync>("rgbd_sync", qos_settings);
   
        _initializeServers();

        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        status = Heartbeat::STOPPED;



    _client = this->create_client<custom_interfaces::srv::DataStoreRgbdSyncInsert>("rgbd_sync_insert");


   

    }

    void RgbdSynchronizer::initNode()
    {
        status = Heartbeat::STARTING;

        status = Heartbeat::RUNNING;
    }

    void RgbdSynchronizer::shutDownNode()
    {
        status = Heartbeat::STOPPING;

        status = Heartbeat::STOPPED;
    }

    void RgbdSynchronizer::_initializeServers()
    {
        _action_server = rclcpp_action::create_server<Action>(
            this,
            "rgbd_sync",
            std::bind(&RgbdSynchronizer::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RgbdSynchronizer::_handleCancel, this, std::placeholders::_1),
            std::bind(&RgbdSynchronizer::_handleAccepted, this, std::placeholders::_1));
    }

    rclcpp_action::GoalResponse RgbdSynchronizer::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, Action::Goal::ConstSharedPtr /*goal*/)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse RgbdSynchronizer::_handleCancel(const std::shared_ptr<GoalHandle> /*goal_handle*/)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void RgbdSynchronizer::_handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread{std::bind(&RgbdSynchronizer::_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void RgbdSynchronizer::_execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        helpers::Timer timer("Scene publisher action", get_logger());
        auto result = std::make_shared<Action::Result>();
        if (status != Heartbeat::RUNNING)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Node is not in running state");
            goal_handle->abort(result);
            return;
        }
        // auto request_timestamp = goal_handle->get_goal()->timestamp;
        auto abortReturn = [this, &goal_handle, &result]()
        {
            _rgbd_sync_publisher->publish(RgbdSync());

            goal_handle->abort(result);
        };

        // Validate input message
        if (!_rgb_images_data || _rgb_images_data->header.stamp == builtin_interfaces::msg::Time() || 
            !_depth_images_data || _depth_images_data->header.stamp == builtin_interfaces::msg::Time())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid input message. Goal failed.");
            abortReturn();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing scene data");
        _cant_touch_this = true;
        RgbdSync::UniquePtr rgbd_sync= _prepareOutputMessages(_rgb_images_data, _depth_images_data);



    auto request = std::make_shared<custom_interfaces::srv::DataStoreRgbdSyncInsert::Request>();
        while (!_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

    auto data_store_result = _client->async_send_request(request);
        _rgbd_sync_publisher->publish(std::move(rgbd_sync));
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface() , data_store_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Data saved in to DataStorage");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to save data in to DataStorage");
    }
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

    }

    RgbdSync::UniquePtr  RgbdSynchronizer::_prepareOutputMessages(const RgbImages::SharedPtr &rgb_images, const DepthImages::SharedPtr &depth_images)
{
        std_msgs::msg::Header header;
        header.stamp = now();

        RgbdSync::UniquePtr rgbd_sync = std::make_unique<RgbdSync>();
        rgbd_sync->rgb = *rgb_images;
        rgbd_sync->depth = *depth_images;
        _cant_touch_this = false;
        return rgbd_sync;
    }

} // namespace 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rgbd_sync::RgbdSynchronizer)
