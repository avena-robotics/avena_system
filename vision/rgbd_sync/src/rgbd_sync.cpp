#include "rgbd_sync/rgbd_sync.hpp"


namespace rgbd_sync
{
    RgbdSynchronizer::RgbdSynchronizer(const rclcpp::NodeOptions &options)
        : Node("rgbd_sync", options)
    {   

        helpers::commons::setLoggerLevelFromParameter(this);
        _cant_touch_this = false;
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable();

        _rgb_images_sub = create_subscription<custom_interfaces::msg::RgbImages>("rgb_images_stream", qos_settings,
                                                                                       [this](const custom_interfaces::msg::RgbImages::SharedPtr rgb_image_msg) {
                                                                                           if(_cant_touch_this)
                                                                                             return;
                                                                                           _rgb_images_data = rgb_image_msg;
                                                                                       });        
                                                                                       
        _depth_images_sub = create_subscription<custom_interfaces::msg::DepthImages>("depth_images_stream", qos_settings,
                                                                                       [this](const custom_interfaces::msg::DepthImages::SharedPtr depth_image_msg) {
                                                                                           if(_cant_touch_this)
                                                                                             return;
                                                                                           _depth_images_data = depth_image_msg;
                                                                                       });


        _ptclds_sub = create_subscription<custom_interfaces::msg::Ptclds>("ptclds_stream", qos_settings,
                                                                                       [this](const custom_interfaces::msg::Ptclds::SharedPtr ptclds_msg) {
                                                                                           if(_cant_touch_this)
                                                                                             return;
                                                                                           _ptclds_data = ptclds_msg;
                                                                                       });

        qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        // _rgbd_sync_publisher = create_publisher<Response>("rgbd_sync", qos_settings);
   
        _initializeServers();




    _client = this->create_client<custom_interfaces::srv::DataStoreRgbdSyncInsert>("rgbd_sync_insert");

        status = Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");

   

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
            // _rgbd_sync_publisher->publish(Response());

            goal_handle->abort(result);
        };

        // Validate input message
        if (!_rgb_images_data || _rgb_images_data->header.stamp == builtin_interfaces::msg::Time() || 
            !_depth_images_data || _depth_images_data->header.stamp == builtin_interfaces::msg::Time() ||
            !_ptclds_data || _ptclds_data->header.stamp == builtin_interfaces::msg::Time())
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid input message. Goal failed.");
            abortReturn();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing scene data");
        _cant_touch_this = true;
        auto request = _prepareOutputMessages(_rgb_images_data, _depth_images_data, _ptclds_data);

        while (!_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(request->data.ptclds.cam1_ptcld,cloud);
        // helpers::visualization::visualize({cloud});
        // cv::Mat mat;
        // helpers::converters::rosImageToCV(request->data.rgb.cam1_rgb, mat);
        // cv::imshow("dupa", mat);
        // cv::waitKey(1);

    auto data_store_result = _client->async_send_request(request);
    data_store_result.wait_for(5s);
        if (rclcpp::ok())
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }

    }

    Request::SharedPtr RgbdSynchronizer::_prepareOutputMessages(const RgbImages::SharedPtr &rgb_images, const DepthImages::SharedPtr &depth_images, const Ptclds::SharedPtr &ptclds)
    {
        Request::SharedPtr request_msg = std::make_shared<Request>();
        request_msg->data.header.stamp = now();
        request_msg->data.rgb = *rgb_images;
        request_msg->data.depth = *depth_images;
        request_msg->data.ptclds = *ptclds;

        _cant_touch_this = false;
        return request_msg;
    }

} // namespace 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rgbd_sync::RgbdSynchronizer)
