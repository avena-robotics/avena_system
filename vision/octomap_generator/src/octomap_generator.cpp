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
        this->declare_parameter<float>("offset", ROBOT_SELF_FILTER_DEFAULT_OFFSET);

    }

    void OctomapGenerator::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(this->get_logger(), "Initialization of compose items action server.");

        this->_action_server = rclcpp_action::create_server<OctomapGeneratorAction>(
            this,
            "octomap_generator",
            std::bind(&OctomapGenerator::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&OctomapGenerator::_handleCancel, this, std::placeholders::_1),
            std::bind(&OctomapGenerator::_handleAccepted, this, std::placeholders::_1));

        bool _pointCloud_transformer_status;
        _pointCloud_transformer = std::make_shared<ptcld_transformer::PtcldTransformer>(this, _pointCloud_transformer_status);

        std::cout << "_pointCloud_transformer_status" << _pointCloud_transformer_status << std::endl;
        if (!_pointCloud_transformer_status)
            shutDownNode();
        else
        {   
            float robot_self_filter_offset = ROBOT_SELF_FILTER_DEFAULT_OFFSET;
            _robot_self_filter = std::make_shared<robot_self_filter::RobotSelfFilter>(this, robot_self_filter_offset);
            RCLCPP_INFO(this->get_logger(), "OctomapGenerator items action server Inicialized.");

            _rgbd_sync_select_client = this->create_client<custom_interfaces::srv::DataStoreRgbdSyncSelect>("rgbd_sync_select");
            _scene_insert_client = this->create_client<custom_interfaces::srv::DataStoreSceneInsert>("scene_insert");

            status = custom_interfaces::msg::Heartbeat::RUNNING;
        }
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

    void OctomapGenerator::_getData(custom_interfaces::msg::RgbdSync::SharedPtr &cameras_data)
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
            cameras_data->ptclds = rgbd_sync_result.get()->data.ptclds;
            cameras_data->header = rgbd_sync_result.get()->data.header;
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

        custom_interfaces::msg::RgbdSync::SharedPtr cameras_data(new custom_interfaces::msg::RgbdSync);

        _getData(cameras_data);

        if (_validateInputs(cameras_data))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid input message. Goal failed.");
            goal_handle->abort(result);
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // cameras_data->ptclds
        // for(auto &ptcld : cameras_data->ptclds)
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ptclds(cameras_data->ptclds.size());
        for(size_t i=0 ; i<cameras_data->ptclds.size(); i++)
        {
            // pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld(new pcl::PointCloud<pcl::PointXYZ>);
            helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(cameras_data->ptclds[i], ptclds[i]);
            ptclds[i]->header.frame_id = "camera_" + std::to_string(i+1);
            ptclds[i] = _pointCloud_transformer->transfromPointcloud(ptclds[i]);
            *ptcld_filtered += *ptclds[i];

        }

        _robot_self_filter->removeRobotFromCloud(ptcld_filtered);

        if (_sendDataToDB(ptclds, ptcld_filtered) == 0)
        {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
        else
        {
            goal_handle->abort(result);
        }
    }

    int OctomapGenerator::_sendDataToDB(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &ptclds, pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld_filtered)
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
        request->data.transformed_ptclds.ptclds_trans.resize(ptclds.size());
        for(size_t i =0; i<ptclds.size(); i++)
            helpers::converters::pclToRosPtcld<pcl::PointXYZ>(ptclds[i], request->data.transformed_ptclds.ptclds_trans[i]);

        helpers::converters::pclToRosPtcld<pcl::PointXYZ>(ptcld_filtered, request->data.octomap.scene_octomap);
        request->data.header.stamp = now();

        auto data_store_result = _scene_insert_client->async_send_request(request);
        if (rclcpp::ok() && data_store_result.wait_for(5s) == std::future_status::ready)
            return 0;

        return 1;
    }

    int OctomapGenerator::_validateInputs(custom_interfaces::msg::RgbdSync::SharedPtr cameras_data)
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
