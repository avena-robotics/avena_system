#include "ptcld_transformer/ptcld_transformer.hpp"
namespace ptcld_transformer
{
    PtcldTransformer::PtcldTransformer(rclcpp::Node *node)
    {        
        _node = node;
        _cameras_parameters = _getCameraTransform();
        _getTableAreaFromParametersServer();
    }


    void PtcldTransformer::joinThread()
    {
        if (_param_server_read && _param_server_thread.joinable())
        {
            _param_server_thread.join();
            _timer.reset();
        }
    }

    TransformedPointClouds::SharedPtr PtcldTransformer::transfromPointcloud(const custom_interfaces::msg::Ptclds::SharedPtr cameras_data_msg)
    {
        helpers::Timer timer(__func__, _node->get_logger());
        if (!_param_server_read)
            return TransformedPointClouds::SharedPtr();
        helpers::TicTok::GetInstance()->start();
        CamerasData::SharedPtr cameras_data = _readInputData(cameras_data_msg);

        return _processCamerasData(cameras_data, _cameras_parameters);

    }

    TransformedPointClouds::SharedPtr PtcldTransformer::_processCamerasData(CamerasData::SharedPtr cameras_data, CameraTransform::SharedPtr cameras_parameters)
    {
        // helpers::Timer timer(__func__, get_logger());
        TransformedPointClouds::SharedPtr transformed_point_clouds(new TransformedPointClouds);

        pcl::PointCloud<pcl::PointXYZ>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp2(new pcl::PointCloud<pcl::PointXYZ>);

        if(cameras_parameters->cam1_aff.matrix() != Eigen::Affine3f().matrix() || cameras_parameters->cam2_aff.matrix() != Eigen::Affine3f().matrix()  ){

        std::thread t3(std::bind(&PtcldTransformer::_filterAndTransformPointCloud, this, cameras_data->cam1_ptcld, cameras_parameters->cam1_aff, temp1));
        std::thread t4(std::bind(&PtcldTransformer::_filterAndTransformPointCloud, this, cameras_data->cam2_ptcld, cameras_parameters->cam2_aff, temp2));
        t3.join();
        t4.join();
        *transformed_point_clouds->merged_ptcld += *temp1;
        *transformed_point_clouds->merged_ptcld += *temp2;

        }else{
            RCLCPP_WARN_STREAM(_node->get_logger(), "there is no valid camera pose, cant transform pointCloud");
 
        }


        return transformed_point_clouds;
    }

    void PtcldTransformer::_filterAndTransformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, const Eigen::Affine3f &camera_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud)
    {
        helpers::vision::voxelize(in_cloud, 0.005);
        pcl::transformPointCloud(*in_cloud, *out_cloud, camera_pose); // transform point cloud to world frame

        pcl::PointXYZ min;
        min.getVector3fMap() = Eigen::Vector3f(_table_area.x_min, _table_area.y_min, _table_area.z_min - 0.01);
        pcl::PointXYZ max;
        max.getVector3fMap() = Eigen::Vector3f(_table_area.x_max, _table_area.y_max, _table_area.z_max);
        pcl::CropBox<pcl::PointXYZ> crop_box_table_edge_ext;
        crop_box_table_edge_ext.setInputCloud(out_cloud);
        crop_box_table_edge_ext.setMin(min.getVector4fMap());
        crop_box_table_edge_ext.setMax(max.getVector4fMap());
        crop_box_table_edge_ext.filter(*out_cloud);
    }

  CameraTransform::SharedPtr PtcldTransformer::_getCameraTransform()
    {
        // helpers::Timer timer(__func__, get_logger());
        CameraTransform::SharedPtr cameras_parameters(new CameraTransform);
        auto get_camera_affine = [this](std::string camera_frame)
        {
            std::optional<Eigen::Affine3f> camera_affine_opt;
            camera_affine_opt = helpers::vision::getCameraTransformAffine("world", camera_frame);
            if (camera_affine_opt == std::nullopt)
            {
                RCLCPP_WARN(_node->get_logger(), "Ptcld transformer - cannot obtain transform to \"" + camera_frame);
                throw(std::runtime_error(std::string("Ptcld transformer - cannot obtain transform to \"") + camera_frame));
            }
            else
            {
                Eigen::Affine3f cam_aff = *camera_affine_opt;
                return cam_aff;
            }
            Eigen::Affine3f cam_aff;
            return cam_aff;
        };
        cameras_parameters->cam1_aff = get_camera_affine("camera_1");       
        cameras_parameters->cam2_aff = get_camera_affine("camera_2");     

        return cameras_parameters;
    }

    void PtcldTransformer::_getTableAreaFromParametersServer()
    {
        helpers::Timer timer(__func__, _node->get_logger());

        try
        {
            while (rclcpp::ok())
            {
                json areas_parameters = helpers::commons::getParameter("areas");
                // if (!rclcpp::ok())
                //     rclcpp::shutdown();
                if (!areas_parameters.empty())
                {
                    _table_area.x_min = areas_parameters["table_area"]["min"]["x"].get<float>();
                    _table_area.y_min = areas_parameters["table_area"]["min"]["y"].get<float>();
                    _table_area.z_min = areas_parameters["table_area"]["min"]["z"].get<float>();
                    _table_area.x_max = areas_parameters["table_area"]["max"]["x"].get<float>();
                    _table_area.y_max = areas_parameters["table_area"]["max"]["y"].get<float>();
                    _table_area.z_max = areas_parameters["table_area"]["max"]["z"].get<float>();
                    _param_server_read = true;
                    break;
                }

                RCLCPP_WARN_STREAM_THROTTLE(_node->get_logger(), (*_node->get_clock()), 1000, "Ptcld transformer - cannot read parameter from server");
            }
        }
        catch (const json::exception &e)
        {
            std::string error_message = "aaaa Error occured while parsing parameter from the server. Message: " + std::string(e.what()) + ". Exiting...";
            RCLCPP_FATAL(_node->get_logger(), error_message);
            rclcpp::shutdown();
            throw std::runtime_error(error_message);
        }
        // return table_area;
    }


    CamerasData::SharedPtr PtcldTransformer::_readInputData(const custom_interfaces::msg::Ptclds::SharedPtr cameras_data_msg)
    {
        CamerasData::SharedPtr cameras_data(new CamerasData);

        std::thread t1(std::bind(&PtcldTransformer::_convertCloudToPCL, this, cameras_data_msg->cam1_ptcld, cameras_data->cam1_ptcld));
        std::thread t2(std::bind(&PtcldTransformer::_convertCloudToPCL, this, cameras_data_msg->cam2_ptcld, cameras_data->cam2_ptcld));

        t1.join();
        t2.join();

        return cameras_data;
    }

    void PtcldTransformer::_convertCloudToPCL(const sensor_msgs::msg::PointCloud2 &ros_cloud_msg, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_pcl_cloud)
    {
        helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(ros_cloud_msg, out_pcl_cloud);
    }

    void PtcldTransformer::_convertCloudToRos(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud, sensor_msgs::msg::PointCloud2 &out_ros_cloud_msg)
    {
        // helpers::converters::pclToRosPtcld<pcl::PointXYZ>(transformed_point_clouds->cam1_ptcld_trans, ptcld_transformer_data->cam1_ptcld_trans);
        helpers::converters::pclToRosPtcld<pcl::PointXYZ>(pcl_cloud, out_ros_cloud_msg);
    }

    builtin_interfaces::msg::Time PtcldTransformer::_getMergedPtcldTime(const builtin_interfaces::msg::Time &cam1_stamp, const builtin_interfaces::msg::Time &cam2_stamp)
    {
        double cam1_time_sec = cam1_stamp.sec + cam1_stamp.nanosec / 1e9;
        double cam2_time_sec = cam2_stamp.sec + cam2_stamp.nanosec / 1e9;
        double merged_time_sec = 0;
        if (cam1_time_sec > cam2_time_sec)
            merged_time_sec = cam2_time_sec + (cam1_time_sec - cam2_time_sec) / 2.0;
        else
            merged_time_sec = cam1_time_sec + (cam2_time_sec - cam1_time_sec) / 2.0;
        builtin_interfaces::msg::Time merged_time_msg;
        merged_time_msg.sec = std::floor(merged_time_sec);
        merged_time_msg.nanosec = (merged_time_sec - merged_time_msg.sec) * 1e9;
        return merged_time_msg;
    }

} // namespace ptcld_transformer

