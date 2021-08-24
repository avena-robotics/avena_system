#include "get_cameras_data/get_cameras_data.hpp"

namespace get_cameras_data
{
    GetCamerasData::GetCamerasData(const rclcpp::NodeOptions &options)
        : Node("get_cameras_data", options)
    {
        helpers::commons::setLoggerLevelFromParameter(this);

        rclcpp::QoS qos_setting = rclcpp::QoS(rclcpp::KeepLast(2)).durability_volatile().reliable();

        _rgb1_image_sub.subscribe(this, _camera1_frame + _rgb_topic, qos_setting.get_rmw_qos_profile());
        _depth1_image_sub.subscribe(this, _camera1_frame + _depth_topic, qos_setting.get_rmw_qos_profile());
        _rgb2_image_sub.subscribe(this, _camera2_frame + _rgb_topic, qos_setting.get_rmw_qos_profile());
        _depth2_image_sub.subscribe(this, _camera2_frame + _depth_topic, qos_setting.get_rmw_qos_profile());

        // _ptclds_sub = create_subscription<custom_interfaces::msg::Ptclds>(
        //     _camera1_frame + _ptcld_topic, qos_setting.get_rmw_qos_profile(), [this](const custom_interfaces::msg::Ptclds::SharedPtr ptclds) {

        //     });
        _ptclds_pub = create_publisher<custom_interfaces::msg::Ptclds>("ptclds_stream", qos_setting);
        _ptclds_sub = create_subscription<sensor_msgs::msg::PointCloud2>(_camera1_frame + _ptcld_topic, qos_setting,
                                                                         [this](const sensor_msgs::msg::PointCloud2::SharedPtr mgs)
                                                                         {
                                                                             custom_interfaces::msg::Ptclds ptclds;
                                                                             //  ptclds.header.frame_id = ""
                                                                             ptclds.cam1_ptcld = *mgs;
                                                                             ptclds.cam2_ptcld = *mgs;
                                                                             ptclds.cam1_ptcld.header.frame_id = "camera_1/rgb_camera_link";
                                                                             ptclds.cam2_ptcld.header.frame_id = "camera_2/rgb_camera_link";
                                                                             ptclds.header.stamp = mgs->header.stamp;
                                                                             _ptclds_pub->publish(ptclds);
                                                                         });

        _syncApproximate = std::make_unique<Synchronizer>(SyncPolicy(2),
                                                          _rgb1_image_sub, _depth1_image_sub,
                                                          _rgb2_image_sub, _depth2_image_sub);
        _syncApproximate->registerCallback(std::bind(&GetCamerasData::_synchronizedTopicsCallback, this,
                                                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                                                     std::placeholders::_4));

        _rgb_images_pub = create_publisher<custom_interfaces::msg::RgbImages>("rgb_images_stream", qos_setting);
        _depth_images_pub = create_publisher<custom_interfaces::msg::DepthImages>("depth_images_stream", qos_setting);
        _ptclds_pub = create_publisher<custom_interfaces::msg::Ptclds>("ptclds_stream", qos_setting);

        // _robot_links_names = helpers::commons::getRobotLinksNames(this);
    }

    GetCamerasData::~GetCamerasData()
    {
    }

    void GetCamerasData::_synchronizedTopicsCallback(const sensor_msgs::msg::Image::ConstSharedPtr &cam1_rgb, const sensor_msgs::msg::Image::ConstSharedPtr &cam1_depth,
                                                     const sensor_msgs::msg::Image::ConstSharedPtr &cam2_rgb, const sensor_msgs::msg::Image::ConstSharedPtr &cam2_depth)
    {
        // RCLCPP_WARN_STREAM(get_logger(), "START: " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << " ms");

        helpers::Timer timer("GetCamerasData::synchronizedCallback", get_logger());

        custom_interfaces::msg::RgbImages::UniquePtr rgb_images_data(new custom_interfaces::msg::RgbImages);
        custom_interfaces::msg::DepthImages::UniquePtr depth_images_data(new custom_interfaces::msg::DepthImages);

        auto timestamp = now();

        rgb_images_data->header.stamp = timestamp;
        sensor_msgs::msg::Image cam1_rgb_output, cam2_rgb_output;
        auto convertBGRAtoBGR = [](const sensor_msgs::msg::Image::ConstSharedPtr &bgra_input, sensor_msgs::msg::Image &bgr_output)
        {
            cv::Mat mat;
            // std::cout << "before: " << bgra_input->encoding << std::endl;
            helpers::converters::rosImageToCV(*bgra_input, mat);
            cv::cvtColor(mat, mat, CV_BGRA2BGR);
            // std::cout << "after 1: " << mat.type() << std::endl;
            helpers::converters::cvMatToRos(mat, bgr_output);

            // std::cout << "after 2: " << bgr_output.encoding << std::endl;
            // cv::imshow("image", mat);
            // cv::waitKey(1);
        };
        convertBGRAtoBGR(cam1_rgb, cam1_rgb_output);
        convertBGRAtoBGR(cam2_rgb, cam2_rgb_output);


        rgb_images_data->cam1_rgb = cam1_rgb_output;
        rgb_images_data->cam2_rgb = cam2_rgb_output;
        rgb_images_data->cam1_rgb.header.frame_id = "camera_1/rgb_camera_link";
        rgb_images_data->cam2_rgb.header.frame_id = "camera_2/rgb_camera_link";

        depth_images_data->header.stamp = timestamp;
        depth_images_data->cam1_depth = *cam1_depth;
        depth_images_data->cam2_depth = *cam2_depth;

        depth_images_data->cam1_depth.header.frame_id = "camera_1/rgb_camera_link";
        depth_images_data->cam2_depth.header.frame_id = "camera_2/rgb_camera_link";
        {
            helpers::Timer timer("GetCamerasData::publish", get_logger());
            // cv::Mat mat;
            // helpers::converters::rosImageToCV(rgb_images_data->cam1_rgb, mat);
            // cv::imshow("dupa", mat);
            // cv::waitKey(1);


            _rgb_images_pub->publish(std::move(rgb_images_data));
            _depth_images_pub->publish(std::move(depth_images_data));
        }
    }

} // namespace get_cameras_data

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(get_cameras_data::GetCamerasData)
