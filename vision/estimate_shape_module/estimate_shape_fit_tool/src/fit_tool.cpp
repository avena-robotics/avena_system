#include "estimate_shape_fit_tool/fit_tool.hpp"


//fits box to handle to obtain initial tool handle position 
//compare colors of visable sides to obtain handle rotation

namespace estimate_shape
{
    FitTool::FitTool(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters)
    {
        _hue_treshold = 10;
    }

    FitTool::~FitTool()
    {
    }

    int FitTool::fit(Item &item)
    {
        Timer timer("FitTool::fit", LOGGER);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto &el : item.item_elements)
            *temp_cloud += *el.pcl_merged;

        // auto vis = helpers::vision::visualize({temp_cloud}, {}, nullptr, "box_cloud");
        // _sortParts(item_elements);
        _fitBox(item);

        _fixBoxEstimation(item);

        if (_fixRotation(item))
            return 1;

        return 0;
    }

    int FitTool::_getHue(pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud, float &hue)
    {
        if (hsv_cloud->points.size() == 0)
            return 1;
        std::map<int, int> colors_counter;
        for (auto pt : hsv_cloud->points)
            colors_counter[static_cast<int>(pt.h)]++;

        int currentMax = 0;
        int arg_max = 0;
        for (auto it = colors_counter.cbegin(); it != colors_counter.cend(); ++it)
        {
            if (it->second > currentMax)
            {
                arg_max = it->first;
                currentMax = it->second;
            }
        }

        hue = arg_max;
        return 0;
    }

    int FitTool::_fixBoxEstimation(Item &item)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_merged(new pcl::PointCloud<pcl::PointXYZRGB>);
        for(auto &rgb_cloud : item.item_elements[HANDLE].pclds_rgb)
            *handle_merged = *rgb_cloud;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        _extractPlane(handle_merged, plane_cloud);

        pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloudXYZRGBtoXYZHSV(*plane_cloud, *hsv_cloud);
        float hue;
        if (_getHue(hsv_cloud, hue))
            return 1;

        Eigen::Vector3f position_vec(item.pose.translation());
        pcl::PointXYZ plane_center;
        pcl::computeCentroid(*plane_cloud, plane_center);
        Eigen::Vector3f center_shift = position_vec - plane_center.getVector3fMap();
        center_shift.normalize();

        if ((hue > YELLOW - _hue_treshold && hue < YELLOW + _hue_treshold) ||
            (hue > CYAN - _hue_treshold && hue < CYAN + _hue_treshold))
        {
            float shift = 9.7750e-02 / 2;
            center_shift *= shift;
        }
        else
        {
            float shift = 4.8876e-02 / 2;
            center_shift *= shift;
        }
        position_vec = plane_center.getVector3fMap() + center_shift;
        item.pose = Eigen::Translation3f(position_vec) * item.pose.rotation();
        // position = helpers::vision::assignJsonFromPosition(position_vec);

        return 0;
    }

    int FitTool::_extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_plane_cloud)
    {

        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.005);
        seg.setInputCloud(cloud_in);
        seg.segment(*inliers, *plane_coefficients);

        if (inliers->indices.size() > 0)
        {
            pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            extract.setInputCloud(cloud_in);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*out_plane_cloud);
            extract.setNegative(true);
            extract.filter(*cloud_in);
        }
        return 0;
    }

    int FitTool::_fixRotation(Item &item)
    {

        Eigen::Affine3f handle_transform = item.pose;
        Eigen::Quaternionf orient(item.pose.rotation());
        Eigen::Vector3f position(item.pose.translation());

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr handle_merged(new pcl::PointCloud<pcl::PointXYZRGB>);
        // *handle_merged = *item.item_elements[HANDLE].pcl_merged;
        // helpers::converters::rosPtcldtoPcl<pcl::PointXYZRGB>(item.item_elements[HANDLE].element_pcl_rgb, handle_merged);
        for(auto &rgb_cloud: item.item_elements[HANDLE].pclds_rgb)
            *handle_merged = *rgb_cloud;

        pcl::transformPointCloud(*handle_merged, *handle_merged, handle_transform.inverse());

        if (_alignAxisWithColor(handle_merged, orient))
            return 1;

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr test(new pcl::PointCloud<pcl::PointXYZRGB>);
        // *test += *item.item_elements[HANDLE].element_pcl_1;
        // *test += *item.item_elements[HANDLE].element_pcl_2;
        // Eigen::Affine3f test_trans = Eigen::Translation3f(position) * orient;

        // pcl::transformPointCloud(*test, *test, test_trans.inverse());

        item.pose = Eigen::Translation3f(item.pose.translation()) * orient;
        // item_elements[HANDLE].orientation = helpers::vision::assignJsonFromQuaternion(orient);

        return 0;
    }

    /**
     * @brief gets direction vector of a wall and its coloc hue, extracting plane, and returning remaining Ptcld
     * 
     * @param handle_merged CAUTION! this method will extract plane out of cloud
     * @param direction_vec vector aligned with axis of an object pointing towards wall.
     * @param hue color hue
     * @return int 
     */
    int FitTool::_getDirectionVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &handle_merged, Eigen::Vector3f &direction_vec, float &hue)
    {

        direction_vec = Eigen::Vector3f(0, 0, 0);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        _extractPlane(handle_merged, plane_cloud);



        pcl::PointXYZ plane_center;
        pcl::computeCentroid(*plane_cloud, plane_center);
        std::vector<float> xyz = {abs(plane_center.x), abs(plane_center.y), abs(plane_center.z)};
        int max_idx = (std::max_element(xyz.begin(), xyz.end()) - xyz.begin());
        direction_vec[max_idx] = plane_center.getVector3fMap()[max_idx];
        direction_vec.normalize();
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr hsv_cloud(new pcl::PointCloud<pcl::PointXYZHSV>);
        pcl::PointCloudXYZRGBtoXYZHSV(*plane_cloud, *hsv_cloud);

        if (_getHue(hsv_cloud, hue))
            return 1;

        return 0;
    }

    int FitTool::_alignAxisWithColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &handle_merged, Eigen::Quaternionf &orient)
    {
        std::string axis;
        Eigen::Vector3f direction_transformed;
        Eigen::Vector3f direction_vec;
        float hue;

        _getDirectionVector(handle_merged, direction_vec, hue);
        Eigen::Quaternionf rot_fix;
        if (hue > MAGENTA - _hue_treshold && hue < MAGENTA + _hue_treshold)
        {
            axis = "y";
            rot_fix.setFromTwoVectors(direction_vec, Eigen::Vector3f::UnitY());
        }
        else if (hue > BLUE - _hue_treshold && hue < BLUE + _hue_treshold)
        {
            axis = "y";
            rot_fix.setFromTwoVectors(direction_vec, -Eigen::Vector3f::UnitY());
        }
        else if (hue > LIME - _hue_treshold && hue < LIME + _hue_treshold)
        {
            axis = "x";
            rot_fix.setFromTwoVectors(direction_vec, Eigen::Vector3f::UnitX());
        }
        else if (hue > RED - _hue_treshold && hue < RED + _hue_treshold)
        {
            axis = "x";
            rot_fix.setFromTwoVectors(direction_vec, -Eigen::Vector3f::UnitX());
        }
        else if (hue > YELLOW - _hue_treshold && hue < YELLOW + _hue_treshold)
        {
            axis = "z";
            rot_fix.setFromTwoVectors(direction_vec, -Eigen::Vector3f::UnitZ());
        }
        else if (hue > CYAN - _hue_treshold && hue < CYAN + _hue_treshold)
        {
            axis = "z";
            rot_fix.setFromTwoVectors(direction_vec, Eigen::Vector3f::UnitZ());
        }
        else
            return 1;


        orient = (rot_fix * orient.inverse()).inverse();
        Eigen::Affine3f transform = Eigen::Translation3f(0, 0, 0) * rot_fix;
        pcl::transformPointCloud(*handle_merged, *handle_merged, transform);

        _getDirectionVector(handle_merged, direction_vec, hue);
        Eigen::Vector3f proper_axis;

        float sign = 1.0;

        if (hue > RED - _hue_treshold && hue < RED + _hue_treshold)
        {
            sign = -1.0;
            proper_axis = sign * Eigen::Vector3f::UnitX();
        }
        else if (hue > LIME - _hue_treshold && hue < LIME + _hue_treshold)
            proper_axis = sign * Eigen::Vector3f::UnitX();
        else if (hue > BLUE - _hue_treshold && hue < BLUE + _hue_treshold)
        {
            sign = -1.0;
            proper_axis = sign * Eigen::Vector3f::UnitY();
        }
        else if (hue > MAGENTA - _hue_treshold && hue < MAGENTA + _hue_treshold)
            proper_axis = sign * Eigen::Vector3f::UnitY();
        else if (hue > CYAN - _hue_treshold && hue < CYAN + _hue_treshold)
            proper_axis = sign * Eigen::Vector3f::UnitZ();
        else if (hue > YELLOW - _hue_treshold && hue < YELLOW + _hue_treshold)
        {
            sign = -1.0;
            proper_axis = sign * Eigen::Vector3f::UnitZ();
        }

        else
            return 1;

        float angle = acos(direction_vec.dot(proper_axis)) * sign;
        orient = (helpers::vision::rotateAroundAxis(axis, angle) * orient.inverse()).inverse();

        return 0;
    }

    int FitTool::_fitBox(Item &item)
    {
        FitBox fit_box(_camera_params, _args, _default_parameters);
        fit_box.fit(item);
        return 0;
    }
} // namespace estimate_shape
