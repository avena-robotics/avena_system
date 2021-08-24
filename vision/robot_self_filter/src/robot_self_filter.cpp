#include "robot_self_filter/robot_self_filter.hpp"

namespace robot_self_filter
{
    RobotSelfFilter::RobotSelfFilter(rclcpp::Node *node)
    {   
         _node = node;
        _offset = DEFAULT_OFFSET;
        _debug = false;
        RCLCPP_INFO(_node->get_logger(), "Initialization of robot self filter.");
        _node->declare_parameter<float>("offset", DEFAULT_OFFSET);
        _loadAvenaMeshes();


        _transforms_buffer = std::make_unique<tf2_ros::Buffer>(_node->get_clock(), tf2::Duration(std::chrono::seconds(5)));
        _transform_listener = std::make_unique<tf2_ros::TransformListener>(*_transforms_buffer);

 
        RCLCPP_INFO(_node->get_logger(), "...done");
    }



    void RobotSelfFilter::removeRobotFromCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr merged_ptcld)
    {
  
        helpers::Timer timer(__func__, _node->get_logger());
        _loadAvenaMeshes();
        if(_robot_links_names.size() == 0)
            return;
        _removeRobotFromPointCloud(merged_ptcld);

    }

    void RobotSelfFilter::_removeRobotFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud)
    {
        // helpers::Timer timer(__func__, get_logger());
        std::set<int> robot_indices_unique;
        _node->get_parameter_or("offset", _offset, DEFAULT_OFFSET);
        std::chrono::microseconds timestamp(point_cloud->header.stamp);
        int index = 0;
        for (auto link :_robot_links_names)
        {

            Eigen::Affine3f aff;
            int transform_check = _lookupTransform(point_cloud->header.frame_id, link, timestamp, aff);


            std::vector<int> indices;
            pcl::CropBox<pcl::PointXYZ> crop_box;
            crop_box.setInputCloud(point_cloud);
            pcl::PointXYZ padded_min_pt;
            padded_min_pt.x = _meshes[link].min_pt.x - _offset;
            padded_min_pt.y = _meshes[link].min_pt.y - _offset;
            padded_min_pt.z = _meshes[link].min_pt.z - _offset;
            pcl::PointXYZ padded_max_pt;
            padded_max_pt.x = _meshes[link].max_pt.x + _offset;
            padded_max_pt.y = _meshes[link].max_pt.y + _offset;
            padded_max_pt.z = _meshes[link].max_pt.z + _offset;
            crop_box.setMin(padded_min_pt.getVector4fMap());
            crop_box.setMax(padded_max_pt.getVector4fMap());
            if (transform_check == 0)
                crop_box.setTransform(aff.inverse());
            else{
                std::cout << point_cloud->header.frame_id << " - to - " <<  link << std::endl;
                RCLCPP_WARN(_node->get_logger(), "There was problem with robot transform. Make sure to pass proper robot TF.");
            }
            crop_box.filter(indices);
            robot_indices_unique.insert(indices.begin(), indices.end());
            index++;
        }
        std::vector<int> robot_indices;
        robot_indices.reserve(robot_indices.size());
        robot_indices.insert(robot_indices.end(), robot_indices_unique.begin(), robot_indices_unique.end());
        helpers::vision::extract(robot_indices, true, point_cloud);
    }

    int RobotSelfFilter::_lookupTransform(const std::string &target_frame, const std::string &source_frame, const std::chrono::microseconds &timestamp, Eigen::Affine3f &out_transform)
    {
        // helpers::Timer timer(__func__, get_logger());
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            auto time = rclcpp::Time(std::chrono::nanoseconds(timestamp).count());
            // auto time = rclcpp::Time(0);
            rclcpp::Duration duration(std::chrono::milliseconds(10));
            transform_stamped = _transforms_buffer->lookupTransform(target_frame, source_frame, time, duration);
            helpers::converters::geometryToEigenAffine(transform_stamped.transform, out_transform);
        }
        catch (const tf2::TransformException &err)
        {
            RCLCPP_DEBUG(_node->get_logger(), err.what());
            return 1;
        }
        return 0;
    }


    std::vector<std::string> RobotSelfFilter::_removeRobotPrefix(std::vector<std::string> link_names, std::string robot_prefix){

        std::vector<std::string> links_without_prefix(link_names.size());
        for(size_t i=0; i<link_names.size();i++)
            links_without_prefix[i] = link_names[i].substr((robot_prefix.length()+1),link_names[i].length()-robot_prefix.length()-1);
        return links_without_prefix;
    }

    void RobotSelfFilter::_loadAvenaMeshes()
    {
        if(_robot_links_names.size() != 0)
            return;
      nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (!parameters.empty())
         return;
        
        const std::string working_side = parameters["working_side"];
        if (auto robot_info = helpers::commons::getRobotInfo(working_side))
            _robot_info = *robot_info;
        else
            return;

        _robot_links_names = _robot_info.link_names;
        _robot_links_names.insert(_robot_links_names.end(),_robot_info.gripper_info.link_names.begin(), _robot_info.gripper_info.link_names.end());

        std::vector<std::string> robot_links_names_to_load = _removeRobotPrefix(_robot_info.link_names, _robot_info.robot_prefix);
        std::vector<std::string> gripper_link_names_to_load = _removeRobotPrefix(_robot_info.gripper_info.link_names, _robot_info.robot_prefix);
        robot_links_names_to_load.insert(robot_links_names_to_load.end(),gripper_link_names_to_load.begin(),gripper_link_names_to_load.end());

        std::string meshes_directory = ament_index_cpp::get_package_share_directory("avena_bringup") + "/" + "robot_links" + "/" + _robot_info.robot_name + "/";
       
        for(size_t i=0;i<_robot_links_names.size(); i++)
        {   
            std::string path = meshes_directory + robot_links_names_to_load[i] + ".ply";
            pcl::PLYReader reader;
            reader.read(path, _meshes[_robot_links_names[i]].mesh);
            pcl::fromPCLPointCloud2(_meshes[_robot_links_names[i]].mesh.cloud, *_meshes[_robot_links_names[i]].cloud);
            _meshes[_robot_links_names[i]].cloud->header.frame_id = _robot_links_names[i];

            _meshes[_robot_links_names[i]].min_pt.x = _meshes[_robot_links_names[i]].cloud->getMatrixXfMap().row(0).minCoeff();
            _meshes[_robot_links_names[i]].min_pt.y = _meshes[_robot_links_names[i]].cloud->getMatrixXfMap().row(1).minCoeff();
            _meshes[_robot_links_names[i]].min_pt.z = _meshes[_robot_links_names[i]].cloud->getMatrixXfMap().row(2).minCoeff();

            _meshes[_robot_links_names[i]].max_pt.x = _meshes[_robot_links_names[i]].cloud->getMatrixXfMap().row(0).maxCoeff();
            _meshes[_robot_links_names[i]].max_pt.y = _meshes[_robot_links_names[i]].cloud->getMatrixXfMap().row(1).maxCoeff();
            _meshes[_robot_links_names[i]].max_pt.z = _meshes[_robot_links_names[i]].cloud->getMatrixXfMap().row(2).maxCoeff();
        }

    }

} // namespace robot_self_filter

// #include "rclcpp_components/register_node_macro.hpp"
// RCLCPP_COMPONENTS_REGISTER_NODE(robot_self_filter::RobotSelfFilter)
