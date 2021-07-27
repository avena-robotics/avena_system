#ifdef TESTS
#include <gtest/gtest.h>
#endif
#include "pcl_methods/create_ptcld.hpp"


namespace robot
{
    CreatePtcld::CreatePtcld()
    {
    }

    int CreatePtcld::cutDepthMapWithMask(cv::Mat &mask, cv::Mat &depth, cv::Mat &out_item_depth, bool reversed)
    {
        if (mask.empty() || depth.empty())
            return 1;
        if (reversed)
        {
            cv::Mat rev_mask = 255 - mask;
            depth.copyTo(out_item_depth, rev_mask);
        }
        else
        {
            depth.copyTo(out_item_depth, mask);
        }
        return 0;
    }

    int CreatePtcld::setCameraParams(transform_map &transform, std::map<std::string, CameraParameters> &cam_params)
    {
        _camera_parameters = cam_params;
        _camera_transform = transform;
        return 0;
    }

    int CreatePtcld::compareItemsBetweenCameras(cv::Mat &reconstructed_mask, std::string label, std::vector<item_cam_t> &detections_cam2, uint32_t &out_corresponding_item_id)
    {
        cv::Mat result;
        for (uint32_t i = 0; i < detections_cam2.size(); i++)
        {
            if (detections_cam2[i].label != label)
            {
                continue;
            }
            cv::Mat res;
            cv::bitwise_and(reconstructed_mask, detections_cam2[i].mask, res);
            int size;
            int size_res;
            size_res = cv::countNonZero(res);
            size = cv::countNonZero(reconstructed_mask);
            if (size_res > 0.2 * size)
            {
                out_corresponding_item_id = i;
                return 0;
            }
        }
        return 1;
    }

    int CreatePtcld::findCorespondingItem(item_cam_t &item, std::vector<item_cam_t> &detections_cam2, uint32_t &out_corresponding_item_id)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // createPtcld(mask_depth_cam1, true, 1, out_item_cloud);
        transformCloudToFrame(item.pcl_data, _frames.camera_frame2, transformed_cloud);
        cv::Mat reconstructed_mask;
        convertPtcldToMask(transformed_cloud, reconstructed_mask);
        return compareItemsBetweenCameras(reconstructed_mask, item.label, detections_cam2, out_corresponding_item_id);
    }

    int CreatePtcld::passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string axis, float min_limit, float max_limit)
    {
        if (cloud->points.size() == 0)
            return 1;
        pcl::PassThrough<pcl::PointXYZRGB> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(axis);
        pass.setFilterLimits(min_limit, max_limit);
        pass.setFilterLimitsNegative(false);
        pass.filter(*cloud);
        return 0;
    }

    int CreatePtcld::voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, float leaf_size)
    {
        if (cloud->points.size() == 0)
            return 1;
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        // voxel_filter.setMinimumPointsNumberPerVoxel(5);
        voxel_filter.filter(*out_cloud);
        return 0;
    }

    int clusterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::vector<pcl::PointIndices> &out_cluster_indices, float cluster_tolerance, int min_cluster_size, int max_cluster_size)
    {
        if (cloud->points.size() == 0)
            return 1;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
        tree->setInputCloud(cloud);
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
        ec.setClusterTolerance(cluster_tolerance); // 2cm
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(out_cluster_indices);
        return 0;
    }

    int CreatePtcld::transformCloudToFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
    {
        std::string cloud_frame = cloud->header.frame_id;
        Eigen::Affine3f transform = Eigen::Affine3f::Identity(); // = _camera_transform[camera_frame];
        if (cloud_frame == "world")
        {
            transform = _camera_transform[frame].inverse();
        }
        else
        {
            transform = _camera_transform[cloud_frame];
        }
        pcl::transformPointCloud(*cloud, *cloud_out, transform.matrix(), true);
        cloud_out->header.frame_id = frame;
        return 0;
    }

    int CreatePtcld::computeBB(cv::Mat &mask, ROI &out_roi)
    {
        cv::Mat Points;
        cv::findNonZero(mask, Points);
        cv::Rect Min_Rect = cv::boundingRect(Points);
        //TODO test what if points is empty (black picture as mask)
        out_roi.x = Min_Rect.x;
        out_roi.y = Min_Rect.y;
        out_roi.dx = Min_Rect.width;
        out_roi.dy = Min_Rect.height;
        return 0;
    }

    bool CreatePtcld::removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcld, pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_ptcld)
    {
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        // build the filter
        outrem.setInputCloud(ptcld);
        outrem.setRadiusSearch(0.005);
        outrem.setMinNeighborsInRadius(4);
        // apply filter
        outrem.filter(*filtered_ptcld);
        return true;
    }

    int CreatePtcld::reconstructPointCloud(WorkspaceArea workspace_area, cv::Mat &depth_image, cv::Mat &rgb, CameraParameters cam_params, ROI roi, std::string frame, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shadow)
    {
        // helpers::Timer timer ("reconstructing ptcld",true);
        if (cloud_out == nullptr)
            cloud_out = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        cloud_out->header.frame_id = frame; // assign camera frame to object cloud
        cloud_out->height = roi.dy;
        cloud_out->width = roi.dx;
        cloud_out->points.resize(roi.dy * roi.dx);
        pcl::PointXYZRGB point;
        cv::resize(rgb, rgb, depth_image.size(), 0, 0, cv::INTER_NEAREST);

        float x_scale = static_cast<float>(depth_image.cols) / static_cast<float>(cam_params.width);
        float y_scale = static_cast<float>(depth_image.rows) / static_cast<float>(cam_params.height);
        cam_params.cx *= x_scale;
        cam_params.cy *= y_scale;
        cam_params.fx *= x_scale;
        cam_params.fy *= y_scale;
        {
            for (int i = roi.x; i < (roi.x + roi.dx); i++)
            {
                for (int j = roi.y; j < (roi.y + roi.dy); j++)
                {
                    // Get Z value of 3D point

                    // Compute XY of 3D point from 2D depth masks using camera intriniscs
                    float z = depth_image.at<float>(j, i);
                    if (z == 0 || z >= workspace_area.camera_max_distance)
                        point.z = NAN;
                    else
                        point.z = z;
                    point.x = point.z * (i - cam_params.cx) / cam_params.fx;
                    point.y = point.z * (j - cam_params.cy) / cam_params.fy;
                    cv::Vec3b rgb_value = rgb.at<cv::Vec3b>(j, i);
                    point.b = rgb_value[0];
                    point.g = rgb_value[1];
                    point.r = rgb_value[2];
                    cloud_out->points[(j - roi.y) * cloud_out->width + i - roi.x] = point;
                }
            }
        }
        if (workspace_area.data_type == "real")
        {
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGB>);
            // *cloud_input = *cloud_out;

            // {
            // helpers::Timer timer("Bilateral filter", true);
            pcl::FastBilateralFilterOMP<pcl::PointXYZRGB>::Ptr bilateral(new pcl::FastBilateralFilterOMP<pcl::PointXYZRGB>);
            bilateral->setNumberOfThreads(std::thread::hardware_concurrency() / 2);
            bilateral->setInputCloud(cloud_out);
            bilateral->setSigmaR(0.5);
            bilateral->setSigmaS(1);
            bilateral->filter(*cloud_out);
            // }

            pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);

            // {
            // helpers::Timer timer("Normal average 3d gradient ptr", true);
            pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointNormal>::Ptr ne(new pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointNormal>);
            ne->setNormalEstimationMethod(ne->COVARIANCE_MATRIX);
            ne->setMaxDepthChangeFactor(0.01f);
            ne->setBorderPolicy(ne->BORDER_POLICY_MIRROR);
            // ne->setDepthDependentSmoothing(true);
            ne->setNormalSmoothingSize(4.0f);
            ne->setInputCloud(cloud_out);
            ne->compute(*normals);
            // }
            // ros2 bag play real_secene_0.db3 --qos-profile-overrides-path /root/ros2_ws/src/avena_ros2/system_tests/scripts/policy_override.yaml -l

            // ros2 run tf2_ros static_transform_publisher 0.628, -1.002, 0.842 0.677, 0.657, 0.134, -0.305 world camera_2/rgb_camera_link

            // ros2 run tf2_ros static_transform_publisher 0.679, 1.011, 0.753 0.512, 0.809, -0.286, 0.043 world camera_1/rgb_camera_link

            // ros2 run fake_joint_state_publisher fake_joint_state_publisher

            float treshold = 0.35; //0.49
            pcl::PointIndices::Ptr shadow_indices(new pcl::PointIndices);
            auto remove_shadows = [=](float treshold, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &input_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud, pcl::PointIndices::Ptr &pi)
            {
                // helpers::Timer tiemr("shadows: ", true);
                pcl::ShadowPoints<pcl::PointXYZRGB, pcl::PointNormal>::Ptr shadow_extraction(new pcl::ShadowPoints<pcl::PointXYZRGB, pcl::PointNormal>(true));
                shadow_extraction->setInputCloud(input_cloud);
                shadow_extraction->setThreshold(treshold);
                shadow_extraction->setNormals(input_normals);
                shadow_extraction->filter(*output_cloud);
                shadow_extraction->getRemovedIndices(*pi);
            };
            // we need original poincloud to extract shadows
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_extract(new pcl::PointCloud<pcl::PointXYZRGB>);
            *cloud_out_extract = *cloud_out;
            remove_shadows(treshold, cloud_out, normals, cloud_out, shadow_indices);
            // std::cout << "Shadow removal " << cloud_out->points.size() << std::endl;

            if (shadow_indices->indices.size() > 0)
            {
                // std::cout << "Extract indices.size: " << shadow_indices->indices.size() << std::endl;
                pcl::ExtractIndices<pcl::PointXYZRGB>::Ptr extract(new pcl::ExtractIndices<pcl::PointXYZRGB>);
                extract->setInputCloud(cloud_out_extract);
                extract->setIndices(shadow_indices);
                extract->setNegative(false);
                extract->filter(*cloud_shadow);
            }

            pcl::Indices index;
            pcl::removeNaNFromPointCloud(*cloud_out, *cloud_out, index);
            // std::cout << "Without nans " << cloud_out->points.size() << std::endl;

            // std::cout << "With nans shadow " << cloud_shadow->points.size() << std::endl;
            pcl::Indices index_s;
            pcl::removeNaNFromPointCloud(*cloud_shadow, *cloud_shadow, index_s);
            // std::cout << "Without nans shadow " << cloud_shadow->points.size() << std::endl;

            int error_code = transformCloudToFrame(cloud_out, "world", cloud_out);
            int error_code2 = transformCloudToFrame(cloud_shadow, "world", cloud_shadow);

            pcl::PassThrough<pcl::PointXYZRGB>::Ptr pass(new pcl::PassThrough<pcl::PointXYZRGB>);
            pass->setInputCloud(cloud_out);
            pass->setFilterFieldName("x");
            pass->setFilterLimits(workspace_area.x_min, workspace_area.x_max);
            pass->setFilterLimitsNegative(false);
            pass->filter(*cloud_out);
            pass->setFilterFieldName("y");
            pass->setFilterLimits(workspace_area.y_min, workspace_area.y_max);
            pass->filter(*cloud_out);
            pass->setFilterFieldName("z");
            pass->setFilterLimits(workspace_area.z_min, workspace_area.z_max);
            pass->filter(*cloud_out);

            // ///////////////////////////////

            bool visualize = false;
            if (visualize)
            {
                pcl::MomentOfInertiaEstimation<pcl::PointXYZRGB> feature_extractor;
                feature_extractor.setInputCloud(cloud_out);
                feature_extractor.compute();

                std::vector<float> moment_of_inertia;
                std::vector<float> eccentricity;
                pcl::PointXYZRGB min_point_OBB;
                pcl::PointXYZRGB max_point_OBB;
                pcl::PointXYZRGB position_OBB;
                Eigen::Matrix3f rotational_matrix_OBB;
                float major_value, middle_value, minor_value;
                Eigen::Vector3f major_vector, middle_vector, minor_vector;
                Eigen::Vector3f mass_center;

                feature_extractor.getMomentOfInertia(moment_of_inertia);
                feature_extractor.getEccentricity(eccentricity);
                feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
                feature_extractor.getEigenValues(major_value, middle_value, minor_value);
                feature_extractor.getEigenVectors(major_vector, middle_vector, minor_vector);
                feature_extractor.getMassCenter(mass_center);
                pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
                viewer->setBackgroundColor(0, 0, 0);
                // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal>(cloud_out_extract, normals, 10, 0.04, "normals", 0);

                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_out, "filtered_cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered_cloud");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "filtered_cloud");

                viewer->addPointCloud<pcl::PointXYZRGB>(cloud_shadow, "shadow");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "shadow");
                viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "shadow");
                viewer->addCoordinateSystem(0.1);
                Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
                Eigen::Quaternionf quat(rotational_matrix_OBB);
                viewer->addCube(position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
                std::cout<< "X dimension : " <<max_point_OBB.x - min_point_OBB.x<<std::endl;
                std::cout<< "Y dimension : " <<max_point_OBB.y - min_point_OBB.y<<std::endl;
                std::cout<< "Z dimension : " <<max_point_OBB.z - min_point_OBB.z<<std::endl;

                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

                pcl::PointXYZ center(mass_center(0), mass_center(1), mass_center(2));
                pcl::PointXYZ x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
                pcl::PointXYZ y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
                pcl::PointXYZ z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
                viewer->addLine(center, x_axis, 0.3f, 0.0f, 0.0f, "major eigen vector"); //main ingredient
                viewer->addLine(center, y_axis, 0.0f, 0.3f, 0.0f, "middle eigen vector");
                viewer->addLine(center, z_axis, 0.0f, 0.0f, 0.3f, "minor eigen vector");
                viewer->spin();
            }

            return error_code + error_code2;
        }
        else
        {
            statisticalOutlierRemovalFilter(cloud_out);
            int error_code = transformCloudToFrame(cloud_out, "world", cloud_out);

            pcl::PassThrough<pcl::PointXYZRGB>::Ptr pass(new pcl::PassThrough<pcl::PointXYZRGB>);
            pass->setInputCloud(cloud_out);
            pass->setFilterFieldName("x");
            pass->setFilterLimits(workspace_area.x_min, workspace_area.x_max);
            pass->setFilterLimitsNegative(false);
            pass->filter(*cloud_out);
            pass->setFilterFieldName("y");
            pass->setFilterLimits(workspace_area.y_min, workspace_area.y_max);
            pass->filter(*cloud_out);
            pass->setFilterFieldName("z");
            pass->setFilterLimits(workspace_area.z_min, workspace_area.z_max);
            pass->filter(*cloud_out);

            return error_code;
        }
    }

    int CreatePtcld::radiusOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptcld, float radius, size_t neighbors)
    {
        if (ptcld->size() == 0)
            return 1;
        pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
        outrem.setInputCloud(ptcld);
        outrem.setRadiusSearch(radius);
        outrem.setMinNeighborsInRadius(neighbors);
        outrem.filter(*ptcld);
        return 0;
    }

    int CreatePtcld::statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud)
    {
        if (out_cloud->points.size() == 0)
            return 1;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
        filter.setInputCloud(out_cloud);
        filter.setMeanK(150);
        filter.setStddevMulThresh(1.96);
        filter.filter(*out_cloud);
        return 0;
    }
    int CreatePtcld::convertPtcldToMask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, cv::Mat &out_mat, float res_scale)
    {
        CameraParameters cam_params = _camera_parameters["camera_2"];
        cam_params.cx *= res_scale;
        cam_params.cy *= res_scale;
        cam_params.fx *= res_scale;
        cam_params.fy *= res_scale;
        cam_params.width *= res_scale;
        cam_params.height *= res_scale;
        //TODO test  if mat.create with zeros is faster
        out_mat = cv::Mat::zeros(cam_params.height, cam_params.width, CV_8UC1);
        for (auto it = cloud->points.begin(); it != cloud->points.end(); it++)
        {
            size_t x = (it->x * cam_params.fx) / it->z + cam_params.cx;
            size_t y = (it->y * cam_params.fy) / it->z + cam_params.cy;
            if (x < cam_params.width && y < cam_params.height)
                out_mat.at<uchar>(y, x) = 255;
        }
        return 0;
    }

    int CreatePtcld::createPtcld(WorkspaceArea workspace_area, cv::Mat &mask, cv::Mat &depth, cv::Mat &rgb, int camera_index, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_shadow)
    {
        if (cloud_out.get() == nullptr)
            cloud_out = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        ROI roi;
        computeBB(mask, roi);

        CameraParameters cam_params;
        std::string frame;
        switch (camera_index)
        {
        case 1:
            frame = _frames.camera_frame;
            cam_params = _camera_parameters["camera_1"];
            break;
        case 2:
            frame = _frames.camera_frame2;
            cam_params = _camera_parameters["camera_2"];
            break;
        default:
            frame = _frames.camera_frame;
            cam_params = _camera_parameters["camera_1"];
            break;
        }

        int error_code = reconstructPointCloud(workspace_area, depth, rgb, cam_params, roi, frame, cloud_out, cloud_shadow);
        voxelize(cloud_out, cloud_out, 0.005);
        return error_code;
    }
} // namespace robot
