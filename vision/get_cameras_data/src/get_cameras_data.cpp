#include "get_cameras_data/get_cameras_data.hpp"

namespace get_cameras_data
{
    GetCamerasData::GetCamerasData(const rclcpp::NodeOptions &options)
        : Node("get_cameras_data", options)
    {

         status = custom_interfaces::msg::Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        helpers::commons::setLoggerLevelFromParameter(this);
        RCLCPP_INFO(this->get_logger(), "started Node");

    }

    GetCamerasData::~GetCamerasData()
    {
    }

    void GetCamerasData::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");

        auto parameters = helpers::commons::getParameters({"cameras"});
        if (parameters.empty())
            RCLCPP_INFO(get_logger(), "cant read parameters from server...");
        else
        {
            _cameras_amount = parameters["cameras"]["cameras_amount"];
            RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        }

    }

    void GetCamerasData::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        rclcpp::QoS qos_setting = rclcpp::QoS(rclcpp::KeepLast(2)).durability_volatile().reliable();
        _cameras_amount = 0;
        _getParametersFromServer();

        

        //create sycn_aprox object for rgb images
        _rgb_image_subs.resize(_cameras_amount);
        for(size_t i =1; i<=_cameras_amount ; i++){
            std::string topic_name = _camera_frame_prefix + std::to_string(i) + _rgb_topic;
            _rgb_image_subs[i-1] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
            _rgb_image_subs[i-1]->subscribe(this, topic_name, qos_setting.get_rmw_qos_profile());
        }
        _sync_rgb = std::make_shared<synchronizers_image::Images>(_cameras_amount, this, &_rgb_image_subs,"rgb_images_stream");


        //create sycn_aprox object for depth images
        _depth_image_subs.resize(_cameras_amount);
        for(size_t i =1; i<=_cameras_amount ; i++){
            std::string topic_name = _camera_frame_prefix + std::to_string(i) + _depth_topic;
            _depth_image_subs[i-1] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
            _depth_image_subs[i-1]->subscribe(this, topic_name, qos_setting.get_rmw_qos_profile());
        }
        _sync_depth = std::make_shared<synchronizers_image::Images>(_cameras_amount, this, &_depth_image_subs,"depth_images_stream");

        //create sycn_aprox object for ptclds
        _ptcld_subs.resize(_cameras_amount);
        for(size_t i =1; i<=_cameras_amount ; i++){
            std::string topic_name = _camera_frame_prefix + std::to_string(i) + _ptcld_topic;
            _ptcld_subs[i-1] = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
            _ptcld_subs[i-1]->subscribe(this, topic_name, qos_setting.get_rmw_qos_profile());
        }
        _sync_ptcld = std::make_shared<synchronizers_ptcld::Ptclds>(_cameras_amount, this, &_ptcld_subs,"ptclds_stream");


        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }


        void GetCamerasData::shutDownNode()
    {   

        if(_sync_rgb){
            for(auto &sub:_rgb_image_subs)
                sub->unsubscribe();
            _sync_rgb.reset();
        }
        if(_sync_depth){
            for(auto &sub:_depth_image_subs)
                sub->unsubscribe();
            _sync_depth.reset();
        }
        if(_sync_ptcld){
            for(auto &sub:_ptcld_subs)
                sub->unsubscribe();
            _sync_ptcld.reset();
        }

            
        RCLCPP_INFO(this->get_logger(), "shut Down Node");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
    }


} // namespace get_cameras_data

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(get_cameras_data::GetCamerasData)
