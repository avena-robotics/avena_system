#include <calibrate.hpp>

Calibrator::Calibrator()
    : Node("calibrate")
{
    subscription_camera1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_1/points2", 1, std::bind(&Calibrator::camera_1_callback, this, std::placeholders::_1));
    subscription_camera2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "camera_2/points2", 1, std::bind(&Calibrator::camera_2_callback, this, std::placeholders::_1));
    _tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    _can_join = false;
    main_logic_thread = std::thread(std::bind(&Calibrator::_waitForKeyPress, this));
    _join_check_timer = this->create_wall_timer(100ms, std::bind(&Calibrator::_joinWhenFinished, this));
}

void Calibrator::_joinWhenFinished()
{
    if (_can_join && main_logic_thread.joinable())
    {

        main_logic_thread.join();
        _join_check_timer->cancel();
        rclcpp::shutdown();
    }
}

void Calibrator::_waitForKeyPress()
{

    while (_cam2cam_transforms.size() < AMOUNT_OF_SAMPLES)
    {
        std::cout << "Press enter to take a spample" << std::endl;
        std::cin.get();
        _execute();
    }
    std::cout << "Place calibration board in origin and press enter" << std::endl;
    std::cin.get();
    _execute();
    _can_join = true;
}

void Calibrator::_execute()
{
    builtin_interfaces::msg::Time timestamp = now();

    auto findTransform = [this](const sensor_msgs::msg::PointCloud2::SharedPtr &msg) -> Eigen::Affine3f
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        helpers::converters::rosPtcldtoPcl<pcl::PointXYZRGB>(*msg, cloud);
        Eigen::Affine3f transform;
        findTransformFromPointcloud(cloud, transform, msg->header.frame_id);

        return transform;
    };

    while ((camera1_pcd == nullptr) ||
           (camera2_pcd == nullptr) ||
           (camera1_pcd->header.stamp.sec < timestamp.sec) ||
           (camera1_pcd->header.stamp.sec < timestamp.sec))
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for new msgs");
    }

    Eigen::Affine3f camera_1_to_base = findTransform(camera1_pcd);
    Eigen::Affine3f camera_2_to_base = findTransform(camera2_pcd);

    if (
        camera_1_to_base.matrix() == Eigen::Affine3f::Identity().matrix() ||
        camera_2_to_base.matrix() == Eigen::Affine3f::Identity().matrix())
    {
        return;
    }

    RCLCPP_INFO(get_logger(), "Both transform saved");

    Eigen::Affine3f camera_2_to_camera_1 = camera_1_to_base.inverse() * camera_2_to_base;

    if (_cam2cam_transforms.size() < AMOUNT_OF_SAMPLES)
    {
        RCLCPP_INFO(get_logger(), "Taking sample no.%d from %d", _cam2cam_transforms.size() + 1, AMOUNT_OF_SAMPLES);
        _cam2cam_transforms.push_back(camera_2_to_camera_1);
    }
    else
    {
        _publishAvg(camera_1_to_base, camera_2_to_base);
    }
}

void Calibrator::_publishAvg(Eigen::Affine3f &camera_1_to_base, Eigen::Affine3f &camera_2_to_base)
{

    Eigen::Matrix4f sum_matrix = Eigen::Matrix4f::Zero();
    for (const auto &trans : _cam2cam_transforms)
    {
        sum_matrix += trans.matrix();
    }
    Eigen::Affine3f cam2_to_cam1;
    cam2_to_cam1.matrix() = (sum_matrix / _cam2cam_transforms.size());

    camera_1_to_base.matrix() = (camera_1_to_base.matrix() + (camera_2_to_base * cam2_to_cam1.inverse()).matrix()) / 2;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new  pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZ>);

    if (!camera1_pcd || !camera2_pcd)
    {
        std::cout << "no ptcld data, try again" << std::endl;
        return;
    }

    helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(*camera1_pcd, cloud1);

    pcl::transformPointCloud(*cloud1, *cloud1, camera_1_to_base);

    *cloud_merged = *cloud1;
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    findPlane(cloud_merged, plane_coefficients);

    helpers::vision::passThroughFilter(cloud_merged, "y", -0.01, 0.01);
    helpers::vision::passThroughFilter(cloud_merged, "x", -0.60, 0.3);

    helpers::vision::passThroughFilter(cloud_merged, "y", -1.0, 1.0);
    cloud_merged->getMatrixXfMap().row(2) = Eigen::VectorXf::Zero(cloud_merged->points.size());
    cloud_merged->getMatrixXfMap().row(1) = Eigen::VectorXf::Zero(cloud_merged->points.size());

    float table_x_size = 0.805;
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud_merged, centroid);

    pcl::PointCloud<pcl::PointXYZ>::Ptr center(new pcl::PointCloud<pcl::PointXYZ>);
    center->points.push_back(centroid);

    float distance = centroid.x;

    camera_1_to_base.translation().x() += (abs(distance) + (table_x_size / 2));

    Eigen::Vector3f plane_axis(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);
    if (plane_coefficients->values[2] < 0)
        plane_axis = plane_axis * -1;

    Eigen::Quaternionf rotation_to_plane;
    rotation_to_plane.setFromTwoVectors(plane_axis, Eigen::Vector3f::UnitZ());

    Eigen::Quaternionf camera_1_rot(camera_1_to_base.rotation());
    Eigen::Vector3f camera_1_position = camera_1_to_base.translation();

    camera_1_rot = rotation_to_plane * camera_1_rot;

    camera_1_to_base = Eigen::Translation3f(camera_1_position) * camera_1_rot;

    camera_2_to_base = camera_1_to_base * cam2_to_cam1;

    _displayTransform(camera_1_to_base, "world", camera1_pcd->header.frame_id);
    _displayTransform(camera_2_to_base, "world", camera2_pcd->header.frame_id);

    saveToYaml(camera_1_to_base, "world", camera1_pcd->header.frame_id, "camera1.yaml");
    saveToYaml(camera_2_to_base, "world", camera2_pcd->header.frame_id, "camera2.yaml");
}

void Calibrator::saveToYaml(Eigen::Affine3f &camera_transform, std::string parent, std::string child, std::string filename)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_extrinsics_calibration");
    package_share_directory += "/config/";
    boost::filesystem::create_directories(package_share_directory);
    auto file = boost::filesystem::ofstream(package_share_directory + filename);
    auto config = YAML::LoadFile(package_share_directory + filename);
    config["position"].push_back(camera_transform.translation().x());
    config["position"].push_back(camera_transform.translation().y());
    config["position"].push_back(camera_transform.translation().z());
    Eigen::Quaternionf rotation(camera_transform.rotation());
    config["orientation"].push_back(rotation.x());
    config["orientation"].push_back(rotation.y());
    config["orientation"].push_back(rotation.z());
    config["orientation"].push_back(rotation.w());
    config["parent"] = parent;
    config["child"] = child;
    file << config;
    file.close();
}

void Calibrator::findPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{

    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *plane_coefficients);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
}

void Calibrator::findPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients::Ptr plane_coefficients)
{

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *plane_coefficients);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud);
}

void Calibrator::publishTransform(const Eigen::Affine3f &in_transform, const std::string &parent,
                                     const std::string &child)
{

    geometry_msgs::msg::TransformStamped transformStamped;

    transformStamped.header.stamp = now();
    transformStamped.header.frame_id = parent;
    transformStamped.child_frame_id = child;
    transformStamped.transform.translation.x = in_transform.translation().x();
    transformStamped.transform.translation.y = in_transform.translation().y();
    transformStamped.transform.translation.z = in_transform.translation().z();
    Eigen::Quaternionf quat(in_transform.rotation());
    transformStamped.transform.rotation.x = quat.x();
    transformStamped.transform.rotation.y = quat.y();
    transformStamped.transform.rotation.z = quat.z();
    transformStamped.transform.rotation.w = quat.w();
    // broadcaster.sendTransform(transformStamped);
    _tf_broadcaster->sendTransform(transformStamped);
}

void Calibrator::findTransformFromPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::Affine3f &out_trans, std::string camera_name)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr whole_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    *whole_cloud = *cloud;
    findPlane(cloud);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcl::PointCloudXYZRGBtoXYZHSV(*cloud, *hsv_cloud);
    std::sort(hsv_cloud->points.begin(), hsv_cloud->points.end(), [](pcl::PointXYZHSV &a, pcl::PointXYZHSV &b)
              { return a.h < b.h; });

    auto find_color = [this](int COLOR, pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud, float sturation_min, float sturation_max)
    {
        auto begin = std::find_if(hsv_cloud->points.begin(), hsv_cloud->points.end(), [this, COLOR](pcl::PointXYZHSV &a)
                                  { return a.h > COLOR - THRESH; });
        auto end = std::find_if(hsv_cloud->points.begin(), hsv_cloud->points.end(), [this, COLOR](pcl::PointXYZHSV &a)
                                { return a.h > COLOR + THRESH; });
        int begin_idx = begin - hsv_cloud->points.begin();
        int end_idx = end - hsv_cloud->points.begin();
        std::sort(begin, end, [](auto a, auto b)
                  { return a.s < b.s; });
        auto copy_begin = hsv_cloud->points.begin() + begin_idx;
        auto copy_end = hsv_cloud->points.begin() + end_idx;

        auto it_begin = std::find_if(copy_begin, copy_end, [sturation_min](auto point)
                                     { return point.s > sturation_min; });

        auto it_end = std::find_if(copy_begin, copy_end, [sturation_max](auto point)
                                   { return point.s > sturation_max; });

        return std::pair<iterator, iterator>(it_begin, it_end);
    };

    auto extract_cloud = [hsv_cloud](std::pair<iterator, iterator> iterators_pair, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
    {
        size_t min = iterators_pair.second - iterators_pair.first;
        out_cloud->points.resize(min);
        for (auto it = iterators_pair.first; it != iterators_pair.second; it++)
        {
            int i = it - iterators_pair.first;
            pcl::PointXYZHSVtoXYZRGB(*it, out_cloud->points[i]);
        }
    };

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr magenta(new pcl::PointCloud<pcl::PointXYZRGB>);
    auto magenta_pair = find_color(MAGENTA, hsv_cloud, 0.4, 1);
    extract_cloud(magenta_pair, magenta);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(magenta);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(magenta);
    ec.extract(cluster_indices);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;

    for (size_t i = 0; i < cluster_indices.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointIndices::Ptr temp_ind(new pcl::PointIndices);
        *temp_ind = cluster_indices[i];
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(magenta);
        extract.setIndices(temp_ind);
        extract.setNegative(false);
        extract.filter(*cloud);
        clouds.push_back(cloud);
    }

    std::vector<pcl::PointXYZ> centroids;
    centroids.resize(clouds.size());

    for (size_t i = 0; i < clouds.size(); i++)
        pcl::computeCentroid(*(clouds[i]), centroids[i]);

    std::vector<P2P> distance_data;

    for (size_t i = 0; i < centroids.size(); i++)
    {
        for (size_t j = 0; j < centroids.size(); j++)
        {
            if (i == j)
                continue;

            distance_data.push_back(
                P2P(centroids[i], centroids[j]));
        }
    }

    auto line_1_it = std::find_if(
        distance_data.begin(),
        distance_data.end(),
        [](P2P line_data)
        {
            return line_data.distance >= SMALLER_DISTANCE - CIRCLE_RADIUS && line_data.distance <= SMALLER_DISTANCE + CIRCLE_RADIUS;
        });

    auto line_2_it = std::find_if(
        distance_data.begin(),
        distance_data.end(),
        [](P2P line_data)
        {
            return line_data.distance >= BIGGER_DISTANCE - CIRCLE_RADIUS && line_data.distance <= BIGGER_DISTANCE + CIRCLE_RADIUS;
        });

    pcl::PointXYZ center, p1, p2;

    if (line_1_it == distance_data.end() || line_2_it == distance_data.end())
    {
        RCLCPP_WARN(this->get_logger(), "Can't find points in current frame, make sure calibration board is visible from both cameras and try again");
        out_trans = Eigen::Affine3f::Identity();
        return;
    }

    if (line_1_it->p1.getVector3fMap() == line_2_it->p1.getVector3fMap())
    {
        center = line_1_it->p1;

        p1 = line_1_it->p2;
        p2 = line_2_it->p2;
    }
    else if (line_1_it->p2.getVector3fMap() == line_2_it->p1.getVector3fMap())
    {
        center = line_1_it->p2;
        p1 = line_1_it->p1;
        p2 = line_2_it->p2;
    }
    else if (line_1_it->p1.getVector3fMap() == line_2_it->p2.getVector3fMap())
    {
        center = line_1_it->p1;
        p1 = line_1_it->p2;
        p2 = line_2_it->p1;
    }
    else if (line_1_it->p2.getVector3fMap() == line_2_it->p2.getVector3fMap())
    {
        center = line_1_it->p2;
        p1 = line_1_it->p1;
        p2 = line_2_it->p1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr p1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr p2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr center_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    p1.getVector3fMap() -= center.getVector3fMap();
    p2.getVector3fMap() -= center.getVector3fMap();

    Eigen::Quaternionf rotation;
    rotation.setFromTwoVectors(p1.getVector3fMap(), Eigen::Vector3f::UnitY());
    Eigen::Affine3f y_transform = Eigen::Translation3f(0, 0, 0) * rotation;

    p1 = pcl::transformPoint(p1, y_transform);
    p2 = pcl::transformPoint(p2, y_transform);

    p2.y = 0;
    p2.getVector3fMap().normalize();

    float angle = acos(p2.getVector3fMap().dot(Eigen::Vector3f::UnitX()));

    Eigen::Affine3f check_transform = Eigen::Translation3f(0, 0, 0) * Eigen::Quaternionf(helpers::vision::assignRotationMatrixAroundY(angle));
    p2 = pcl::transformPoint(p2, check_transform);

    float rot_sign = 1.0;
    if ((p2.getVector3fMap() - Eigen::Vector3f::UnitX()).norm() > 0.02)
    {
        rot_sign = -1.0;
    }

    rotation = Eigen::Quaternionf(helpers::vision::assignRotationMatrixAroundY(rot_sign * angle)) * rotation;

    Eigen::Affine3f trans = Eigen::Translation3f(center.getVector3fMap()) * rotation.inverse();

    _displayTransform(trans.inverse(), "world", camera_name);
    publishTransform(trans.inverse(), "world", camera_name);

    out_trans = trans.inverse();
}

void Calibrator::_displayTransform(const Eigen::Affine3f &in_transform, const std::string &parent,
                                      const std::string &child)
{
    Eigen::Quaternionf rotation(in_transform.rotation());
    std::cout << "ros2 run tf2_ros static_transform_publisher " << in_transform.translation().x() << " " << in_transform.translation().y() << " " << in_transform.translation().z() << " " << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << " " << parent << " " << child << std::endl;

    publishTransform(in_transform, parent, child);
}

void Calibrator::camera_1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    camera1_pcd = msg;
}

void Calibrator::camera_2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    camera2_pcd = msg;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Calibrator>());
    rclcpp::shutdown();
    return 0;
}
