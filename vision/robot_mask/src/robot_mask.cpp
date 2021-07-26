#include "robot_mask/robot_mask.hpp"

namespace robot_mask
{
    RobotMask::RobotMask(rclcpp::Node *node)
    {
        _node = node;

        _transforms_buffer = std::make_unique<tf2_ros::Buffer>(_node->get_clock(), tf2::Duration(std::chrono::seconds(2)));
        _transform_listener = std::make_unique<tf2_ros::TransformListener>(*_transforms_buffer);

        _loadAvenaMeshes();
        _getCamerasParameters();
    }

    /**
     * @brief prepare robot masks - this function is not blocking so mask are not ready after executing this function.
     * to be sure that process has finished - run waitFormasks function.
     * 
     * @param timestamp 
     * @param out_mask_cam1 
     * @param out_mask_cam2 
     */
    void RobotMask::prepareRobotMask(builtin_interfaces::msg::Time timestamp, cv::Mat &out_mask_cam1, cv::Mat &out_mask_cam2)
    {
        if (_workers_running)
            _joinThreads();
        _workers_running = true;
        if (_robot_links_names.size() == 0 && _loadAvenaMeshes() != 0)
            return;
        if (!_cam_params_loaded)
            _getCamerasParameters();

        out_mask_cam1 = cv::Mat::zeros(cv::Size(_cam_1.second.width, _cam_1.second.height), CV_8UC1);
        out_mask_cam2 = cv::Mat::zeros(cv::Size(_cam_1.second.width, _cam_1.second.height), CV_8UC1);

        _workers.resize(_robot_links_names.size());

        for (size_t i = 0; i < _robot_links_names.size(); i++)
            _workers[i] = std::thread(std::bind(&RobotMask::_getRobotMasks, this, timestamp, _robot_links_names[i], std::ref(out_mask_cam1), std::ref(out_mask_cam2)));
    
    }

    void RobotMask::_joinThreads()
    {
        for (auto &worker : _workers)
            worker.join();
        _workers_running = false;
    }
    /**
     * @brief blocking function - once its  finished, mask passed to prepareRobotMask are ready.
     * 
     */
    void RobotMask::waitForMasks()
    {
        if (!_workers_running)
        {
            RCLCPP_WARN_STREAM(_node->get_logger(), "cannot obtain robot masks, prepareRobotMask function was not called before this function");
            return;
        }
        _joinThreads();
    }

    std::vector<std::string> RobotMask::_removeRobotPrefix(std::vector<std::string> link_names, std::string robot_prefix)
    {

        std::vector<std::string> links_without_prefix(link_names.size());
        for (size_t i = 0; i < link_names.size(); i++)
            links_without_prefix[i] = link_names[i].substr((robot_prefix.length() + 1), link_names[i].length() - robot_prefix.length() - 1);
        return links_without_prefix;
    }

    int RobotMask::_loadAvenaMeshes()
    {
        nlohmann::json parameters = helpers::commons::getParameter("robot");
        if (parameters.empty())
            return 1;
        const std::string working_side = parameters["working_side"];
        _robot_info = helpers::commons::getRobotInfo(working_side);
        _robot_links_names = _robot_info.link_names;
        _robot_links_names.insert(_robot_links_names.end(), _robot_info.gripper_info.link_names.begin(), _robot_info.gripper_info.link_names.end());

        std::vector<std::string> robot_links_names_to_load = _removeRobotPrefix(_robot_info.link_names, _robot_info.robot_prefix);
        std::vector<std::string> gripper_link_names_to_load = _removeRobotPrefix(_robot_info.gripper_info.link_names, _robot_info.robot_prefix);

        robot_links_names_to_load.insert(robot_links_names_to_load.end(), gripper_link_names_to_load.begin(), gripper_link_names_to_load.end());
        std::string meshes_directory = ament_index_cpp::get_package_share_directory("avena_bringup") + "/" + "robot_links" + "/" + _robot_info.robot_name + "/";
        for (size_t i = 0; i < _robot_links_names.size(); i++)
        {
            std::string path = meshes_directory + robot_links_names_to_load[i] + ".ply";
            pcl::PLYReader reader;
            reader.read(path, _meshes[_robot_links_names[i]].mesh);
            pcl::fromPCLPointCloud2(_meshes[_robot_links_names[i]].mesh.cloud, *_meshes[_robot_links_names[i]].cloud);
            _meshes[_robot_links_names[i]].cloud->header.frame_id = _robot_links_names[i];
            helpers::vision::voxelize(_meshes[_robot_links_names[i]].cloud, 0.01);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
            std::vector<pcl::Vertices> polygons;
            pcl::ConcaveHull<pcl::PointXYZ> chull;
            chull.setInputCloud(_meshes[_robot_links_names[i]].cloud);
            chull.setAlpha(0.5);
            chull.reconstruct(*_meshes[_robot_links_names[i]].cloud, _meshes[_robot_links_names[i]].mesh.polygons);
        }
        return 0;
    }

    void RobotMask::_getRobotMasks(const rclcpp::Time timestamp, std::string &link_name, cv::Mat &out_mask_cam1, cv::Mat &out_mask_cam2)
    {

        Eigen::Affine3f link_to_world_transform;
        _lookupTransform(WORLD, link_name, timestamp, link_to_world_transform);

        auto get_link_mask = [this, link_name](camera_info &cam_params, Eigen::Affine3f &link_to_world_transform, cv::Mat &robot_mask)
        {
            Eigen::Affine3f cloud_transform = cam_params.first.inverse() * link_to_world_transform;
            pcl::PointCloud<pcl::PointXYZ>::Ptr link_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud<pcl::PointXYZ>(*(_meshes[link_name].cloud), *link_cloud, cloud_transform);

            std::vector<cv::Point> cv_points(link_cloud->points.size());
            auto cv_it = cv_points.begin();
            for (auto &pt : link_cloud->points)
            {
                cv_it->x = (pt.x * cam_params.second.fx / pt.z) + cam_params.second.cx;
                cv_it->y = (pt.y * cam_params.second.fy / pt.z) + cam_params.second.cy;

                if (cv_it->x < 0)
                    cv_it->x = 0;
                if (cv_it->y < 0)
                    cv_it->y = 0;

                if (cv_it->x > static_cast<int>(cam_params.second.width))
                    cv_it->x = cam_params.second.width;
                if (cv_it->y > static_cast<int>(cam_params.second.height))
                    cv_it->y = cam_params.second.height;
                cv_it++;
            }

            for (auto &group : _meshes[link_name].mesh.polygons)
            {

                std::vector<std::vector<cv::Point>> triangle = {{cv_points[group.vertices[0]], cv_points[group.vertices[1]], cv_points[group.vertices[2]]}};
                if ((triangle[0][0].x == 0 && triangle[0][1].x == 0 && triangle[0][2].x == 0) ||
                    (triangle[0][0].x == cam_params.second.width && triangle[0][1].x == cam_params.second.width && triangle[0][2].x == cam_params.second.width))
                    continue;
                if ((triangle[0][0].y == 0 && triangle[0][1].y == 0 && triangle[0][2].y == 0) ||
                    (triangle[0][0].y == cam_params.second.height && triangle[0][1].y == cam_params.second.height && triangle[0][2].y == cam_params.second.height))
                    continue;
                cv::fillPoly(robot_mask, triangle, cv::Scalar(255));
            }
        };

        get_link_mask(_cam_1, link_to_world_transform, out_mask_cam1);
        get_link_mask(_cam_2, link_to_world_transform, out_mask_cam2);

        return;
    }

    int RobotMask::_getCamerasParameters()
    {
        auto get_camera_parameters = [this](std::string camera_frame)
        {
            std::optional<Eigen::Affine3f> camera_affine_opt;
            while (rclcpp::ok())
            {
                camera_affine_opt = helpers::vision::getCameraTransformAffine("world", camera_frame);
                if (camera_affine_opt)
                    break;
                RCLCPP_WARN_STREAM_THROTTLE(_node->get_logger(), *(_node->get_clock()), 1000, "cannot obtain transform to \"" + camera_frame + "\" trying again...");
            }
            Eigen::Affine3f cam_aff = *camera_affine_opt;

            CameraParameters camera_data;
            while (rclcpp::ok())
            {
                try
                {
                    auto cam_intrinsic = helpers::vision::getCameraIntrinsic(_node->get_node_topics_interface(), camera_frame);
                    if (!cam_intrinsic)
                    {
                        RCLCPP_WARN_STREAM_THROTTLE(_node->get_logger(), *(_node->get_clock()), 1000, "cannot obtain camera intrinsic to \"" + camera_frame + "\" trying again...");
                        continue;
                    }
                    camera_data.width = cam_intrinsic->width;
                    camera_data.height = cam_intrinsic->height;
                    camera_data.cx = cam_intrinsic->cx;
                    camera_data.cy = cam_intrinsic->cy;
                    camera_data.fx = cam_intrinsic->fx;
                    camera_data.fy = cam_intrinsic->fy;
                    camera_data.camera_frame = camera_frame;
                    RCLCPP_INFO(_node->get_logger(), " Succesfully obtained camera parameters");
                    break;
                }
                catch (json::exception &e)
                {
                    RCLCPP_FATAL_STREAM(_node->get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
                    rclcpp::shutdown();
                }
            }

            camera_info camera_pair(cam_aff, camera_data);
            return camera_pair;
        };

        _cam_1 = get_camera_parameters("camera_1");
        _cam_2 = get_camera_parameters("camera_2");
        _cam_params_loaded = true;
        return 0;
    }

    int RobotMask::_lookupTransform(const std::string &target_frame, const std::string &source_frame, const rclcpp::Time &timestamp, Eigen::Affine3f &out_transform)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            rclcpp::Duration duration(std::chrono::milliseconds(1));
            auto time = rclcpp::Time(0);

            // transform_stamped = _transforms_buffer->lookupTransform(target_frame, source_frame, time);
            transform_stamped = _transforms_buffer->lookupTransform(target_frame, source_frame, timestamp, duration);
            helpers::converters::geometryToEigenAffine(transform_stamped.transform, out_transform);
        }
        catch (const tf2::TransformException &err)
        {
            std::cout << err.what() << std::endl;
            RCLCPP_INFO(_node->get_logger(), err.what());
            return 1;
        }
        return 0;
    }

} // namespace robot_mask
