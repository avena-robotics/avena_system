#include "ptcld_transformer/ptcld_transformer.hpp"
namespace ptcld_transformer
{
    PtcldTransformer::PtcldTransformer(rclcpp::Node *node, bool &transformer_status)
    {
        _node = node;

        if (_getCamerasTransform())
        {
            // _cameras_parameters = std::make_shared<CameraTransform>();
            // CameraTransform::SharedPtr cameras_parameters(new CameraTransform);
            // cameras_parameters->cam_aff = *camera_aff_opt;
            // _cameras_parameters = cameras_parameters;

            _getTableAreaFromParametersServer();
            transformer_status = true;
        }
        else
            transformer_status = false;
    }
    PtcldTransformer::~PtcldTransformer(){
        
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PtcldTransformer::transfromPointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld)
    {
        helpers::Timer timer(__func__, _node->get_logger());
        if (!_param_server_read)
            return pcl::PointCloud<pcl::PointXYZ>::Ptr();

        return _processCamerasData(ptcld, _cam_transform[ptcld->header.frame_id]);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr PtcldTransformer::_processCamerasData(const pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld, CameraTransform::SharedPtr cameras_parameters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        if (cameras_parameters->cam_aff.matrix() != Eigen::Affine3f().matrix())
        {
            _filterAndTransformPointCloud(ptcld, cameras_parameters->cam_aff, transformed_point_cloud);
        }
        else
        {
            RCLCPP_WARN_STREAM(_node->get_logger(), "there is no valid camera pose, cant transform pointCloud");
        }

        return transformed_point_cloud;
    }

    void PtcldTransformer::_filterAndTransformPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, const Eigen::Affine3f &camera_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud)
    {
        *out_cloud = *in_cloud;
        helpers::vision::voxelize(out_cloud, 0.005);
        pcl::transformPointCloud(*out_cloud, *out_cloud, camera_pose); // transform point cloud to world frame

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

    bool PtcldTransformer::_getCamerasTransform()
    {

        RCLCPP_INFO_ONCE(_node->get_logger(), "Reading parameters from the server");
        auto parameters = helpers::commons::getParameters({"cameras"});
        if (parameters.empty())
            RCLCPP_INFO(_node->get_logger(), "cant read parameters from server...");
        else
        {
            _cameras_amount = parameters["cameras"]["cameras_amount"];
            RCLCPP_INFO(_node->get_logger(), "Parameters read successfully...");
        }


        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++){
            std::string frame = "camera_" + std::to_string(cam_idx);
            std::optional<Eigen::Affine3f> camera_affine_opt;
             camera_affine_opt = helpers::vision::getCameraTransformAffine("world", frame);
            if (camera_affine_opt == std::nullopt)
                return false;           
             CameraTransform::SharedPtr transform(new CameraTransform);
            transform->cam_aff = *camera_affine_opt;
            _cam_transform[frame] = transform;
        }

        return true;
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
