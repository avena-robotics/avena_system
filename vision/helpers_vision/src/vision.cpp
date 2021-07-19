#include "helpers_vision/vision.hpp"

namespace helpers
{
    namespace vision
    {
        Eigen::Quaternionf rotateAroundAxis(std::string axis_name, float angle)
        {
            if (axis_name == "x")
                return Eigen::Quaternionf(assignRotationMatrixAroundX(angle));
            else if (axis_name == "y")
                return Eigen::Quaternionf(assignRotationMatrixAroundY(angle));
            else if (axis_name == "z")
                return Eigen::Quaternionf(assignRotationMatrixAroundZ(angle));
            else
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("helpers"), "wrong axis name passed: " << axis_name);

            return Eigen::Quaternionf::Identity();
        }

        Eigen::Matrix3f assignRotationMatrixAroundZ(float angle)
        {
            Eigen::Matrix3f y_rot;
            y_rot << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;
            return y_rot;
        }

        Eigen::Matrix3f assignRotationMatrixAroundX(float angle)
        {
            Eigen::Matrix3f y_rot;
            y_rot << 1, 0, 0, 0, cos(angle), -sin(angle), 0, sin(angle), cos(angle);
            return y_rot;
        }

        Eigen::Matrix3f assignRotationMatrixAroundY(float angle)
        {
            Eigen::Matrix3f y_rot;
            y_rot << cos(angle), 0, sin(angle), 0, 1, 0, -sin(angle), 0, cos(angle);
            return y_rot;
        }

        bool checkPointsAmmount(pcl::PointCloud<pcl::PointXYZ>::Ptr &obj_ptcld, size_t k_neighbours)
        {
            pcl::PointIndices::Ptr indices(new pcl::PointIndices);

            for (auto it = obj_ptcld->points.begin(); it != obj_ptcld->points.end(); it++)
            {
                if (!pcl::isFinite(*it))
                {
                    indices->indices.push_back(it - obj_ptcld->points.begin());
                };
            }
            extract(indices, true, obj_ptcld);
            if (obj_ptcld->points.size() < k_neighbours)
            {
                return false;
            }
            return true;
        }

        int extract(pcl::PointIndices::Ptr indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud)
        {
            if (indices->indices.size() == 0)
                return 1;
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(in_cloud);
            extract.setIndices(indices);
            extract.setNegative(set_negative);
            extract.filter(*out_cloud);
            return 0;
        }

        int extract(pcl::PointIndices::Ptr indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_out_cloud)
        {
            return extract(indices, set_negative, in_out_cloud, in_out_cloud);
        }

        int extract(std::vector<int> &indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_out_cloud)
        {
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
            indices_ptr->indices = indices;
            return extract(indices_ptr, set_negative, in_out_cloud);
        }

        int extract(std::vector<int> &indices, bool set_negative, pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud)
        {
            pcl::PointIndices::Ptr indices_ptr(new pcl::PointIndices);
            indices_ptr->indices = indices;
            return extract(indices_ptr, set_negative, in_cloud, out_cloud);
        }

        int rotatePtcldAroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr &obj_ptcld, Eigen::Quaternionf rot, pcl::PointXYZ rot_center, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_ptcld)
        {
            out_ptcld->points.resize(obj_ptcld->points.size());
            for (auto it = obj_ptcld->points.begin(); it != obj_ptcld->points.end(); it++)
            {
                out_ptcld->points[it - obj_ptcld->points.begin()].getVector3fMap() = (rot * (it->getVector3fMap() - rot_center.getVector3fMap())) + rot_center.getVector3fMap();
            }
            return 0;
        }

        int flattenPlaneCloud(Eigen::Quaternionf &z_rot, pcl::PointXYZ &centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_plane_cloud)
        {
            for (auto it = out_plane_cloud->points.begin(); it != out_plane_cloud->points.end(); it++)
            {
                it->getVector3fMap() = it->getVector3fMap() - centroid.getVector3fMap();
                it->getVector3fMap() = z_rot * it->getVector3fMap();
                it->z = 0.0;
            }
            return 0;
        }

        int hull2D(pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &out_hull)
        {
            if (plane_cloud->points.size() == 0)
                return 1;
            pcl::ConvexHull<pcl::PointXYZ> convex_hull;
            convex_hull.setInputCloud(plane_cloud);
            convex_hull.setDimension(2);
            convex_hull.reconstruct(*out_hull);
            return 0;
        }

        Eigen::Matrix3f findRotationFromHullPoints(pcl::PointXYZ &hullPoint, pcl::PointXYZ &followingPoint)
        {
            // For each pair of hull points, determine the angle
            double rise = followingPoint.y - hullPoint.y;
            double run = followingPoint.x - hullPoint.x;
            // and normalize..
            double l = sqrt((rise * rise) + (run * run));
            rise = rise / l;
            run = run / l;
            // Build rotation matrix from change of basis
            Eigen::Matrix3f rotation;
            rotation << run, rise, 0.0, -rise, run, 0.0, 0.0, 0.0, 1.0;
            return rotation;
        }

        int rotateHull(Eigen::Matrix3f &rotation, pcl::PointCloud<pcl::PointXYZ>::Ptr hull, pcl::PointCloud<pcl::PointXYZ>::Ptr out_hull)
        {
            out_hull->points.resize(hull->points.size());
            for (auto it = hull->points.begin(); it != hull->points.end(); it++)
                out_hull->points[it - hull->points.begin()].getVector3fMap() = rotation * it->getVector3fMap();
            return 0;
        }

        int statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud, int mean_k, float std_dev_mul_thresh)
        {
            if (out_cloud->points.size() == 0)
                return 1;
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
            filter.setInputCloud(out_cloud);
            filter.setMeanK(mean_k);
            filter.setStddevMulThresh(std_dev_mul_thresh);
            filter.filter(*out_cloud);
            return 0;
        }

        int medianOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud, int /*window_size*/, float /*window_shift*/)
        {
            if (out_cloud->points.size() == 0)
                return 1;
            // pcl::Indices indices;
            // pcl::PointCloud<pcl::PointIndices> indices_ptcld;
            // pcl::copyPointCloud(*out_cloud,indices_ptcld)
            // pcl::transformPointCloud(*out_cloud,indices_ptcld);
            // pcl::PassThrough<pcl::PointXYZ> pass;
            // pass.setInputCloud(out_cloud);
            // pass.filter(indices);
            // std::cout<<"size :" <<out_cloud->points.size()<<std::endl;
            // pcl::MedianFilter<pcl::Indices> median_filter;
            // median_filter.setInputCloud(indices_ptcld)
            // median_filter.setWindowSize(window_size);
            // median_filter.setMaxAllowedMovement(window_shift);
            // median_filter.filter(indices_ptcld);
            // std::cout<<"size :" <<indices_ptcld.points.size()<<std::endl;

            return 0;
        }

        bool radiusOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld, float radius, size_t neighbors)
        {
            if (ptcld->size() == 0)
                return false;
            pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
            outrem.setInputCloud(ptcld);
            outrem.setRadiusSearch(radius);
            outrem.setMinNeighborsInRadius(neighbors);
            outrem.filter(*ptcld);
            return true;
        }

        int passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::string axis, float min_limit, float max_limit, bool negative)
        {
            if (cloud->points.size() == 0)
                return 1;
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName(axis);
            pass.setFilterLimits(min_limit, max_limit);
            pass.setFilterLimitsNegative(negative);
            pass.filter(*cloud);
            return 0;
        }

        json assignJsonFromPosition(Eigen::Vector3f &position)
        {
            json out_json;
            out_json["x"] = position.x();
            out_json["y"] = position.y();
            out_json["z"] = position.z();
            return out_json;
        }

        json assignJsonFromQuaternion(Eigen::Quaternionf &orientation)
        {
            json out_json;
            out_json["w"] = orientation.w();
            out_json["x"] = orientation.x();
            out_json["y"] = orientation.y();
            out_json["z"] = orientation.z();
            return out_json;
        }

        Eigen::Vector3f assignPositionFromJson(json &position)
        {
            return Eigen::Vector3f(position["x"], position["y"], position["z"]);
        }

        Eigen::Quaternionf assignQuaternionFromJson(json &orientation)
        {
            return Eigen::Quaternionf(orientation["w"], orientation["x"], orientation["y"], orientation["z"]);
        }

        int voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size)
        {
            if (cloud->points.size() == 0)
                return 1;
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(cloud);
            voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
            // voxel_filter.setMinimumPointsNumberPerVoxel(5);
            voxel_filter.filter(*cloud);
            return 0;
        }
        int approximateVoxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float leaf_size)
        {
            if (cloud->points.size() == 0)
                return 1;
            pcl::ApproximateVoxelGrid<pcl::PointXYZ>::Ptr aprox_voxel_filter(new pcl::ApproximateVoxelGrid<pcl::PointXYZ>);
            aprox_voxel_filter->setInputCloud(cloud);
            aprox_voxel_filter->setLeafSize(leaf_size, leaf_size, leaf_size);
            aprox_voxel_filter->filter(*cloud);
            return 0;
        }

        Eigen::Quaternionf computeQuaternionFromAngle(float angle, Eigen::Vector3f reference)
        {
            float roll = reference.x() * angle, pitch = reference.y() * angle, yaw = reference.z() * angle;
            Eigen::Quaternionf rot = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
            return rot;
        }

        std::optional<geometry_msgs::msg::TransformStamped> getTransformStamped(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout)
        {
            geometry_msgs::msg::TransformStamped transform;
            rclcpp::Clock::SharedPtr clock(new rclcpp::Clock);
            tf2_ros::Buffer transforms_buffer(clock);
            tf2_ros::TransformListener transform_listener(transforms_buffer);
            try
            {
                std::chrono::microseconds timeout_microsec = std::chrono::duration_cast<std::chrono::microseconds>(timeout);
                std::string err_message;
                auto start_time = std::chrono::steady_clock::now();
                while (!transforms_buffer.canTransform(target_frame, source_frame, tf2::TimePointZero, tf2::durationFromSec(0), &err_message))
                {
                    if (std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - start_time) > timeout)
                        return std::nullopt;
                }
                transform = transforms_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &err)
            {
                return std::nullopt;
            }
            return transform;
        }

        std::optional<geometry_msgs::msg::TransformStamped> getCameraTransformStamped(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout)
        {
            return getTransformStamped(target_frame, source_frame + "/rgb_camera_link", timeout);
        }

        std::optional<Eigen::Affine3f> getTransformAffine(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout)
        {
            std::optional<geometry_msgs::msg::TransformStamped> transform_geom = getTransformStamped(target_frame, source_frame, timeout);
            if (!transform_geom)
                return std::nullopt;
            Eigen::Translation3f trans(transform_geom->transform.translation.x, transform_geom->transform.translation.y, transform_geom->transform.translation.z);
            Eigen::Quaternionf quat(transform_geom->transform.rotation.w, transform_geom->transform.rotation.x, transform_geom->transform.rotation.y, transform_geom->transform.rotation.z);
            Eigen::Affine3f affine(trans * quat);
            return affine;
        }

        std::optional<Eigen::Affine3f> getCameraTransformAffine(const std::string &target_frame, const std::string &source_frame, const std::chrono::duration<float> &timeout)
        {
            return getTransformAffine(target_frame, source_frame + "/rgb_camera_link", timeout);
        }

        std::optional<CameraIntrinsic> getCameraIntrinsic(rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface, const std::string &camera_frame, const std::chrono::duration<float> &timeout)
        {
            auto sub = rclcpp::create_subscription<sensor_msgs::msg::CameraInfo>(node_topics_interface, camera_frame + "/depth_to_rgb/camera_info", 1, [](const sensor_msgs::msg::CameraInfo::SharedPtr) {});

            sensor_msgs::msg::CameraInfo cam_info;
            rclcpp::MessageInfo info;
            bool message_received = false;
            std::chrono::microseconds timeout_microsec = std::chrono::duration_cast<std::chrono::microseconds>(timeout);
            auto start_time = std::chrono::steady_clock::now();
            while (!message_received && std::chrono::duration<float, std::micro>(std::chrono::steady_clock::now() - start_time) < timeout_microsec)
            {
                message_received = sub->take(cam_info, info);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if (!message_received)
                return std::nullopt;
            CameraIntrinsic cam_intrinsic;
            cam_intrinsic.width = cam_info.width;
            cam_intrinsic.height = cam_info.height;
            cam_intrinsic.cx = cam_info.k[2];
            cam_intrinsic.cy = cam_info.k[5];
            cam_intrinsic.fx = cam_info.k[0];
            cam_intrinsic.fy = cam_info.k[4];
            return cam_intrinsic;
        }
    } // namespace vision

} // namespace helpers
