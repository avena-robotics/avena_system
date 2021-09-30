#ifdef TESTS
#include <gtest/gtest.h>
#endif
#include "pcl_methods/create_ptcld.hpp"

namespace create_ptcld
{

    int cutDepthMapWithMask(cv::Mat &mask, cv::Mat &depth, cv::Mat &out_item_depth, bool reversed)
    {
        if (mask.empty() || depth.empty())
            return 1;
        if (reversed)
        {
            cv::Mat rev_mask = 255 - mask;
            depth.copyTo(out_item_depth, rev_mask);
        }
        else
            depth.copyTo(out_item_depth, mask);
            
        return 0;
    }

    bool compareMasks(cv::Mat &mask1, cv::Mat &mask2, float tresh)
    {
        cv::Mat res(mask1.size(), CV_8UC1);
        cv::bitwise_and(mask2, mask1, res);
        int size;
        int size_res;
        size_res = cv::countNonZero(res);
        size = cv::countNonZero(mask2);

        return (size_res > tresh * size);
    }


    int voxelize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, float leaf_size)
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

    int transformCloudToFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, camera_data_ptr &cams_data, size_t &cam_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
    {
        pcl::transformPointCloud(*cloud, *cloud_out, (*cams_data)[cam_idx].first.inverse().matrix(), true);
        cloud_out->header.frame_id = "camera_" + std::to_string(cam_idx) + "/rgb_camera_link";
        return 0;
    }

    int convertPtcldToMask(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, camera_data_ptr &cams_data, size_t &mask_cam_idx, cv::Mat &out_mat)
    {
        out_mat = cv::Mat::zeros((*cams_data)[mask_cam_idx].second.height, (*cams_data)[mask_cam_idx].second.width, CV_8UC1);
        for (auto it = cloud->points.begin(); it != cloud->points.end(); it++)
        {
            size_t x = (it->x * (*cams_data)[mask_cam_idx].second.fx) / it->z + (*cams_data)[mask_cam_idx].second.cx;
            size_t y = (it->y * (*cams_data)[mask_cam_idx].second.fy) / it->z + (*cams_data)[mask_cam_idx].second.cy;
            if (x < (*cams_data)[mask_cam_idx].second.width && y < (*cams_data)[mask_cam_idx].second.height)
                out_mat.at<uchar>(y, x) = 255;
        }

        return 0;
    }

    int transformCloudToWorldFrame(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, camera_data_ptr &cams_data, size_t &cam_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
    {
        pcl::transformPointCloud(*cloud, *cloud_out, (*cams_data)[cam_idx].first.matrix(), true);
        cloud_out->header.frame_id = "world";
        return 0;
    }

    int computeBB(cv::Mat &mask, ROI &out_roi)
    {
        cv::Mat Points;
        cv::findNonZero(mask, Points);
        cv::Rect Min_Rect = cv::boundingRect(Points);
        out_roi.x = Min_Rect.x;
        out_roi.y = Min_Rect.y;
        out_roi.dx = Min_Rect.width;
        out_roi.dy = Min_Rect.height;
        return 0;
    }

    void computeMaskForIndex(camera_data_ptr &cams_data, size_t idx_of_output_mask, element &element)
    {
        // element._clouds[element.cam_idx];
        if (element.cam_idx == idx_of_output_mask)
            return;

        element._clouds[idx_of_output_mask] = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        transformCloudToFrame(element._clouds[element.cam_idx], cams_data, idx_of_output_mask, element._clouds[idx_of_output_mask]);
        convertPtcldToMask(element._clouds[idx_of_output_mask], cams_data, idx_of_output_mask, element._masks[idx_of_output_mask]);
        //TODO erode generated mask
    }

    void bilateralFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_out_cloud)
    {

        pcl::FastBilateralFilterOMP<pcl::PointXYZRGB>::Ptr bilateral(new pcl::FastBilateralFilterOMP<pcl::PointXYZRGB>);
        bilateral->setNumberOfThreads(std::thread::hardware_concurrency() / 2);
        bilateral->setInputCloud(in_out_cloud);
        bilateral->setSigmaR(0.5);
        bilateral->setSigmaS(1);
        bilateral->filter(*in_out_cloud);
    }

    void normalEstimation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &out_normals)
    {

        pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointNormal>::Ptr ne(new pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::PointNormal>);
        ne->setNormalEstimationMethod(ne->COVARIANCE_MATRIX);
        ne->setMaxDepthChangeFactor(0.01f);
        ne->setBorderPolicy(ne->BORDER_POLICY_MIRROR);
        // ne->setDepthDependentSmoothing(true);
        ne->setNormalSmoothingSize(4.0f);
        ne->setInputCloud(in_cloud);
        ne->compute(*out_normals);
    }

    void filter_shadows(float treshold, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input_cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &input_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output_cloud, pcl::PointIndices::Ptr &pi)
    {
        pcl::ShadowPoints<pcl::PointXYZRGB, pcl::PointNormal>::Ptr shadow_extraction(new pcl::ShadowPoints<pcl::PointXYZRGB, pcl::PointNormal>(true));
        shadow_extraction->setInputCloud(input_cloud);
        shadow_extraction->setThreshold(treshold);
        shadow_extraction->setNormals(input_normals);
        shadow_extraction->filter(*output_cloud);
        shadow_extraction->getRemovedIndices(*pi);
    }

    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &in_out_cloud, WorkspaceArea &workspace_area)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr test = in_out_cloud->makeShared();
        pcl::PassThrough<pcl::PointXYZRGB>::Ptr pass(new pcl::PassThrough<pcl::PointXYZRGB>);
        pass->setInputCloud(test);
        pass->setFilterFieldName("x");
        pass->setFilterLimits(workspace_area.x_min, workspace_area.x_max);
        pass->setFilterLimitsNegative(false);
        pass->filter(*test);
        pass->setFilterFieldName("y");
        pass->setFilterLimits(workspace_area.y_min, workspace_area.y_max);
        pass->filter(*test);
        pass->setFilterFieldName("z");
        pass->setFilterLimits(workspace_area.z_min, workspace_area.z_max);
        pass->filter(*test);
    }

    int reconstructPointCloud(WorkspaceArea &workspace_area, cv::Mat &depth_image, cv::Mat &rgb, camera_data_ptr cams_params, ROI roi, size_t cam_idx, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_out)
    {
        // helpers::Timer timer ("reconstructing ptcld",true);
        //TODO fix for when rgb/ mask and depth resolution do not match - although its stupid cus depth_to_rgb should be rgb res... otherwise mask isnt align with item depth.
        if (cloud_out == nullptr)
            cloud_out = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        cloud_out->header.frame_id = "/camera_" + std::to_string(cam_idx) + "/rgb_camera_link"; // assign camera frame to object cloud

        size_t pixel_idx = 0;
        for (int i = roi.x; i < (roi.x + roi.dx); i++)
        {
            for (int j = roi.y; j < (roi.y + roi.dy); j++)
            {
                // Get Z value of 3D point
                // Compute XY of 3D point from 2D depth masks using camera intriniscs
                float z = depth_image.at<float>(j, i);
                if (z == 0.0f || z >= workspace_area.camera_max_distance)
                    continue;

                cloud_out->points[pixel_idx].z = z;
                cloud_out->points[pixel_idx].x = z * (i - (*cams_params)[cam_idx].second.cx) / (*cams_params)[cam_idx].second.fx;
                cloud_out->points[pixel_idx].y = z * (j - (*cams_params)[cam_idx].second.cy) / (*cams_params)[cam_idx].second.fy;
                cv::Vec3b rgb_value = rgb.at<cv::Vec3b>(j, i);
                cloud_out->points[pixel_idx].b = rgb_value[0];
                cloud_out->points[pixel_idx].g = rgb_value[1];
                cloud_out->points[pixel_idx].r = rgb_value[2];
                pixel_idx++;
            }
        }

        return 0;
    }


    int statisticalOutlierRemovalFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud)
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

    int createPtcld(WorkspaceArea &workspace_area, element &el, cv::Mat &depth, cv::Mat &rgb, size_t camera_index, camera_data_ptr &cams_params, bool remove_shadows)
    {
        if (!el._clouds[el.cam_idx])
            el._clouds[el.cam_idx] = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

        ROI roi;
        computeBB(el._masks[el.cam_idx], roi);
        cv::Mat item_depth;
        cutDepthMapWithMask(el._masks[el.cam_idx], depth, item_depth);

        int error_code = 0;
        error_code += reconstructPointCloud(workspace_area, item_depth, rgb, cams_params, roi, camera_index, el._clouds[el.cam_idx]);

        if (workspace_area.data_type == "real")
        {

            bilateralFilter(el._clouds[el.cam_idx]);

            if (remove_shadows)
            {
                pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
                normalEstimation(el._clouds[el.cam_idx], normals);

                float treshold = 0.35; //0.49
                pcl::PointIndices::Ptr shadow_indices(new pcl::PointIndices);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_extract(new pcl::PointCloud<pcl::PointXYZRGB>);
                *cloud_out_extract = *el._clouds[el.cam_idx];
                filter_shadows(treshold, el._clouds[el.cam_idx], normals, el._clouds[el.cam_idx], shadow_indices);

                if (shadow_indices->indices.size() > 0)
                {
                    pcl::ExtractIndices<pcl::PointXYZRGB>::Ptr extract(new pcl::ExtractIndices<pcl::PointXYZRGB>);
                    extract->setInputCloud(cloud_out_extract);
                    extract->setIndices(shadow_indices);
                    extract->setNegative(false);
                    extract->filter(*el.cloud_shadow);
                }
                pcl::Indices index_s;
                pcl::removeNaNFromPointCloud(*el.cloud_shadow, *el.cloud_shadow, index_s);
                error_code += transformCloudToWorldFrame(el.cloud_shadow, cams_params, camera_index, el.cloud_shadow);
            }
        }

        pcl::Indices index;
        pcl::removeNaNFromPointCloud(*el._clouds[el.cam_idx], *el._clouds[el.cam_idx], index);
        error_code += transformCloudToWorldFrame(el._clouds[el.cam_idx], cams_params, camera_index, el._clouds[el.cam_idx]);

        passThroughFilter(el._clouds[el.cam_idx], workspace_area);
        voxelize(el._clouds[el.cam_idx], el._clouds[el.cam_idx], 0.005);
        error_code += statisticalOutlierRemovalFilter(el._clouds[el.cam_idx]);

        return error_code;
    }

}