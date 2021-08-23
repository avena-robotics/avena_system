#include "octomap_generator.hpp"

namespace octomap_generator
{

    OctomapGenerator::OctomapGenerator(const rclcpp::NodeOptions &options)
        : Node("octomap_generator", options)
    {
        helpers::commons::setLoggerLevelFromParameter(this);

        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), "started Node");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }

    void OctomapGenerator::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(this->get_logger(), "Initialization of compose items action server.");
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)); //.transient_local().reliable();

        this->_action_server = rclcpp_action::create_server<OctomapGeneratorAction>(
            this,
            "octomap_generator",
            std::bind(&OctomapGenerator::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&OctomapGenerator::_handleCancel, this, std::placeholders::_1),
            std::bind(&OctomapGenerator::_handleAccepted, this, std::placeholders::_1));

        _pointCloud_transformer = std::make_shared<ptcld_transformer::PtcldTransformer>(this);
        _robot_self_filter = std::make_shared<robot_self_filter::RobotSelfFilter>(this);
        RCLCPP_INFO(this->get_logger(), "OctomapGenerator items action server Inicialized.");

        _rgbd_sync_select_client = this->create_client<custom_interfaces::srv::DataStoreRgbdSyncSelect>("rgbd_sync_select");
        _scene_insert_client = this->create_client<custom_interfaces::srv::DataStoreSceneInsert>("scene_insert");

        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void OctomapGenerator::shutDownNode()
    {
        RCLCPP_INFO(this->get_logger(), "shut Down Node");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    OctomapGenerator::~OctomapGenerator()
    {
        shutDownNode();
    }

    void OctomapGenerator::_getData(custom_interfaces::msg::Ptclds::SharedPtr &cameras_data)
    {
        auto rgbd_sync_request = std::make_shared<custom_interfaces::srv::DataStoreRgbdSyncSelect::Request>();
        while (!_rgbd_sync_select_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "DataStore RgbdSync service not available, waiting again...");
        }

        auto rgbd_sync_result = _rgbd_sync_select_client->async_send_request(rgbd_sync_request);
        // Wait for the result.
        if (rgbd_sync_result.wait_for(5s) == std::future_status::ready)
        {
            cameras_data->cam1_ptcld = rgbd_sync_result.get()->data.ptclds.cam1_ptcld;
            cameras_data->cam2_ptcld = rgbd_sync_result.get()->data.ptclds.cam2_ptcld;
            cameras_data->header = rgbd_sync_result.get()->data.ptclds.header;
            std::cout << " encoding: " <<  rgbd_sync_result.get()->data.rgb.cam1_rgb.encoding << std::endl;
        }
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to read rgbd_sync data");
    }

    rclcpp_action::GoalResponse OctomapGenerator::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const OctomapGeneratorAction::Goal> /*goal*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse OctomapGenerator::_handleCancel(const std::shared_ptr<GoalHandleOctomapGenerator> /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void OctomapGenerator::_handleAccepted(const std::shared_ptr<GoalHandleOctomapGenerator> goal_handle)
    {
        std::thread{std::bind(&OctomapGenerator::_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void OctomapGenerator::_execute(const std::shared_ptr<GoalHandleOctomapGenerator> goal_handle)
    {
        helpers::Timer timer("Octomap generator action", get_logger());
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto result = std::make_shared<OctomapGeneratorAction::Result>();

        custom_interfaces::msg::Ptclds::SharedPtr cameras_data(new custom_interfaces::msg::Ptclds);

        _getData(cameras_data);

        if (_validateInputs(cameras_data))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid input message. Goal failed.");
            goal_handle->abort(result);
            return;
        }
        ptcld_transformer::TransformedPointClouds::SharedPtr transformer_ptcld = _pointCloud_transformer->transfromPointcloud(cameras_data);
        _robot_self_filter->removeRobotFromCloud(transformer_ptcld->merged_ptcld);

        helpers::visualization::visualize({transformer_ptcld->merged_ptcld});

        if (_sendDataToDB(transformer_ptcld) == 0)
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else
        {
            goal_handle->abort(result);
        }
    }

    int OctomapGenerator::_sendDataToDB(ptcld_transformer::TransformedPointClouds::SharedPtr &octomap_msg)
    {
        auto request = std::make_shared<custom_interfaces::srv::DataStoreSceneInsert::Request>();
        while (!_scene_insert_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *filtered_cloud += *(octomap_msg->cam1_ptcld_trans);
        *filtered_cloud += *(octomap_msg->cam2_ptcld_trans);

        helpers::converters::pclToRosPtcld<pcl::PointXYZ>(filtered_cloud, request->data.transformed_ptclds.merged_ptcld);
        helpers::converters::pclToRosPtcld<pcl::PointXYZ>(octomap_msg->cam1_ptcld_trans, request->data.transformed_ptclds.cam1_ptcld_trans);
        helpers::converters::pclToRosPtcld<pcl::PointXYZ>(octomap_msg->cam2_ptcld_trans, request->data.transformed_ptclds.cam2_ptcld_trans);
        helpers::converters::pclToRosPtcld<pcl::PointXYZ>(octomap_msg->merged_ptcld, request->data.octomap.scene_octomap);

        auto data_store_result = _scene_insert_client->async_send_request(request);
        if (rclcpp::ok() && data_store_result.wait_for(5s) == std::future_status::ready)
            return 0;

        return 1;
    }

    int OctomapGenerator::_validateInputs(custom_interfaces::msg::Ptclds::SharedPtr cameras_data)
    {
        auto checkHeader = [](const std_msgs::msg::Header &header)
        {
            return header.stamp == builtin_interfaces::msg::Time();
        };

        if (!cameras_data || checkHeader(cameras_data->header))
            return 1;
        return 0;
    }

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_generator::OctomapGenerator)
