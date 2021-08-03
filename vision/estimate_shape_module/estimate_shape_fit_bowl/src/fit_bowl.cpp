#include "estimate_shape_fit_bowl/fit_bowl.hpp"

namespace estimate_shape
{
    FitBowl::FitBowl(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters),
          _show(false)
    {
    }

    FitBowl::~FitBowl()
    {
    }

    int FitBowl::fit(Item &item)
    {
        Timer timer("FitBowl::fit", LOGGER);
        std::vector<ItemElement> &item_elements = item.item_elements;
        Eigen::Affine3f pose;
        pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld = item_elements[0].pcl_merged;
        LOG_DEBUG_STREAM("Loading dish pointcloud from database with item_id " << item.id);

        _big_radius = _raw_data["item_description"]["upper_disc_radius"].get<float>();
        _small_radius = _raw_data["item_description"]["lower_disc_radius"].get<float>();
        _height = _raw_data["item_description"]["upper_disc_height"].get<float>();
        float sphere_radius;
        if (_computeSphereRadius(_big_radius, _small_radius, _height, sphere_radius))
            return 1;
        std::cout << "FitBowl:: Sphere estimation radius is equal to :  " << sphere_radius << std::endl;
        pcl::ModelCoefficients coeffients;
        if (_calculateSphereParameters(item_ptcld, coeffients, sphere_radius * 0.9, sphere_radius * 1.1))
            return 1;
        if (_computeInitialPose(item_ptcld, coeffients, _big_radius, pose))
            return 1;
        if (_refineRotation(item_ptcld, pose, _big_radius))
            return 1;
        if (_assignData(item, item_ptcld, pose))
            return 1;
        return 0;
    }

    int FitBowl::_computeSphereRadius(float big_radius, float smaller_radius, float height, float & sphere_radius)
    {
        sphere_radius = std::sqrt(big_radius * big_radius + std::pow((big_radius * big_radius - smaller_radius * smaller_radius - height * height) / (2 * height), 2));
        return 0;
    }

    int FitBowl::_computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Matrix3f &rotational_matrix_OBB, pcl::PointXYZ &position_OBB)
    {
        LOG_DEBUG("FitBowl::_computeOBB: Computing oriented bounding box...");
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;

        if (obj_ptcld->points.size() == 0)
        {
            LOG_WARN("FitBowl::_computeOBB: no points to compute oriented bounding box");
            return 1;
        }
        feature_extractor.setInputCloud(obj_ptcld);
        feature_extractor.compute();
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        return 0;
    }

    int FitBowl::_computeInitialPose(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, pcl::ModelCoefficients &coeffients, float /*radius*/, Eigen::Affine3f &out_pose)
    {

        Eigen::Matrix3f rotational_matrix;
        pcl::PointXYZ position_OBB;
        _computeOBB(item_ptcld, rotational_matrix, position_OBB);

        Eigen::Translation3f initial_translation(coeffients.values[0], coeffients.values[1], coeffients.values[2]);
        pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Quaternionf rotation;
        rotation.setFromTwoVectors(Eigen::Vector3f::UnitZ(), rotational_matrix.col(2));
        Eigen::Affine3f initial_transform = initial_translation * rotation;
        pcl::transformPointCloud(*item_ptcld, *item_ptcld_transformed, initial_transform.inverse());

        pcl::PointXYZ centroid;
        pcl::computeCentroid(*item_ptcld_transformed, centroid);

        if (centroid.z > 0)
        {
            Eigen::Quaternionf rot(helpers::vision::assignRotationMatrixAroundX(M_PI));
            Eigen::Quaternionf old_rot(initial_transform.rotation());
            Eigen::Vector3f temp = initial_transform.translation();
            initial_transform = Eigen::Translation3f(temp) * (old_rot * rot);
        }

        out_pose = initial_transform;

        return 0;
    }

    int FitBowl::_refineRotation(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, Eigen::Affine3f &transform, float radius)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*item_ptcld, *item_ptcld_transformed, transform.inverse());
        float min = item_ptcld_transformed->getMatrixXfMap().row(2).minCoeff();
        helpers::vision::passThroughFilter(item_ptcld_transformed, "z", min, min + 0.02);

        pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.0001);
        seg.setInputCloud(item_ptcld_transformed);
        seg.segment(*inliers, *plane_coefficients);

        Eigen::Vector3f new_z(plane_coefficients->values[0], plane_coefficients->values[1], plane_coefficients->values[2]);

        Eigen::Quaternionf new_rot;
        new_rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), new_z);

        Eigen::Vector3f temp = transform.translation();
        Eigen::Quaternionf old_rot(transform.rotation());
        transform = Eigen::Translation3f(temp) * (old_rot * new_rot);

        pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
        helpers::vision::passThroughFilter(item_ptcld, "z", 0.001, 1);
        pcl::transformPointCloud(*item_ptcld, *item_ptcld_transformed, transform.inverse());

        item_ptcld_transformed->getMatrixXfMap().row(2) - Eigen::VectorXf::Zero(item_ptcld_transformed->points.size());
        helpers::vision::statisticalOutlierRemovalFilter(item_ptcld_transformed, 20, 1.80);
        helpers::vision::hull2D(item_ptcld_transformed, hull);

        temp = transform.translation();

        pcl::PointXYZ center;
        std::sort(hull->points.begin(), hull->points.end(), [](const pcl::PointXYZ &a, const pcl::PointXYZ &b)
                  { return (a.getVector3fMap().norm() < b.getVector3fMap().norm()); });
        center = hull->points[hull->points.size() - 1];
        float center_ofset = center.getVector3fMap().norm() - radius;
        // std::cout << "center_ofset: " << center_ofset << std::endl;
        center.getVector3fMap().normalize();
        center.getVector3fMap() *= center_ofset;

        Eigen::Affine3f position_offset = Eigen::Translation3f(0, 0, 0) * Eigen::Quaternionf(transform.rotation());
        pcl::transformPoint(center, position_offset);

        temp.x() += center.x;
        temp.y() += center.y;
        temp.z() += center.z;

        old_rot = Eigen::Quaternionf(transform.rotation());
        transform = Eigen::Translation3f(temp) * old_rot;
        return 0;
    }

    int FitBowl::_calculateSphereParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, pcl::ModelCoefficients &coeffients, float radius_min, float radius_max)
    {

        pcl::PointIndices inliers;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        try
        {
            seg.setModelType(pcl::SacModel::SACMODEL_SPHERE);
            seg.setMethodType(pcl::SAC_PROSAC);
            seg.setNumberOfThreads(std::thread::hardware_concurrency() / 2);
            seg.setInputCloud(item_ptcld);
            seg.setDistanceThreshold(0.01);
            seg.setOptimizeCoefficients(false);
            seg.setRadiusLimits(radius_min, radius_max);
            seg.setMaxIterations(1000); // 100 000
            seg.segment(inliers, coeffients);
        }
        catch (const std::exception &e)
        {
            LOG_WARN("FitBowl::_computeOBB: no points to compute oriented bounding box. Error: " + std::string(e.what()));
            return 1;
        }

        if (inliers.indices.size() == 0)
            return 1;
        return 0;
    }

    int FitBowl::_assignData(Item &item, pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld, Eigen::Affine3f &pose)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*item_ptcld, *item_ptcld_transformed, pose.inverse());

        helpers::vision::statisticalOutlierRemovalFilter(item_ptcld_transformed);
        pcl::PointCloud<pcl::PointXYZ>::Ptr positions_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        positions_cloud->points.resize(2);

        positions_cloud->points[0].x = 0;
        positions_cloud->points[1].x = 0;
        positions_cloud->points[0].y = 0;
        positions_cloud->points[1].y = 0;
        positions_cloud->points[0].z = item_ptcld_transformed->getMatrixXfMap().row(2).minCoeff();
        positions_cloud->points[1].z = item_ptcld_transformed->getMatrixXfMap().row(2).maxCoeff();

        pcl::transformPointCloud(*positions_cloud, *positions_cloud, pose);

        Eigen::Vector3f smaller_circle_position = positions_cloud->points[0].getVector3fMap();
        Eigen::Vector3f bigger_circle_position = positions_cloud->points[1].getVector3fMap();
        Eigen::Quaternionf bowl_orientation(pose.rotation());

        // Assign bowl pose
        Eigen::Translation3f bowl_position((smaller_circle_position.x() + bigger_circle_position.x()) / 2,
                                           (smaller_circle_position.y() + bigger_circle_position.y()) / 2,
                                           (smaller_circle_position.z() + bigger_circle_position.z()) / 2);
        item.pose = bowl_position * bowl_orientation;

        json part_description;
        part_description["fit_method"] = "bowl";
        part_description["spawn_method"] = "tool";
        part_description["part_name"] = item.label;
        json part_position;
        part_position["x"] = 0.0;
        part_position["y"] = 0.0;
        part_position["z"] = 0.0;
        part_description["pose"]["position"] = part_position;

        json part_orientation;
        part_orientation["x"] = 0.0;
        part_orientation["y"] = 0.0;
        part_orientation["z"] = 0.0;
        part_orientation["w"] = 1.0;
        part_description["pose"]["orientation"] = part_orientation;

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Adding old shit which should not be here (probably)

        const float radius_diff = abs(_big_radius - _small_radius);
        const float out_plate_angle = atan(radius_diff / _height);

        part_description["ramp"]["dims"]["height"] = _height;
        part_description["ramp"]["dims"]["angle"] = out_plate_angle;
        part_description["ramp"]["fit_method"] = "ramp";

        json position_smaller_circle = helpers::vision::assignJsonFromPosition(smaller_circle_position);
        json orientation_smaller_circle = helpers::vision::assignJsonFromQuaternion(bowl_orientation);
        part_description["smaller_circle"]["dims"]["radius"] = _small_radius;
        part_description["smaller_circle"]["fit_method"] = "circle";
        part_description["smaller_circle"]["position"] = position_smaller_circle;
        part_description["smaller_circle"]["orientation"] = orientation_smaller_circle;

        json position_bigger_circle = helpers::vision::assignJsonFromPosition(bigger_circle_position);
        json orientation_bigger_circle = helpers::vision::assignJsonFromQuaternion(bowl_orientation);
        part_description["bigger_circle"]["dims"]["radius"] = _big_radius;
        part_description["bigger_circle"]["fit_method"] = "circle";
        part_description["bigger_circle"]["position"] = position_bigger_circle;
        part_description["bigger_circle"]["orientation"] = orientation_bigger_circle;
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        item.item_elements[0].parts_description = {part_description};
        return 0;
    }

    int FitBowl::checkEstimation(Item &item)
    {
        try
        {
            std::vector<ItemElement> &item_elements = item.item_elements;
            if (item_elements.size() != 1)
                return 1;

            if (item_elements[0].parts_description.size() != 1)
                return 1;

            json &part_description = item_elements[0].parts_description[0];

            auto check_pose = [](const json &pose, const std::string &pose_part_name, const std::vector<std::string> &fields)
            {
                if (!pose.contains(pose_part_name))
                    return 1;
                for (auto &field : fields)
                    if (!pose[pose_part_name].contains(field) || !pose[pose_part_name][field].is_number())
                        return 1;
                return 0;
            };

            if (!part_description.contains("pose"))
                return 1;

            if (check_pose(part_description["pose"], "position", {"x", "y", "z"}))
                return 1;

            if (check_pose(part_description["pose"], "orientation", {"x", "y", "z", "w"}))
                return 1;

            if (!part_description.contains("bigger_circle") ||
                !part_description.contains("smaller_circle") ||
                !part_description.contains("ramp"))
                return 1;

            if (check_pose(part_description["bigger_circle"], "position", {"x", "y", "z"}))
                return 1;

            if (check_pose(part_description["bigger_circle"], "orientation", {"x", "y", "z", "w"}))
                return 1;

            if (check_pose(part_description["smaller_circle"], "position", {"x", "y", "z"}))
                return 1;

            if (check_pose(part_description["smaller_circle"], "orientation", {"x", "y", "z", "w"}))
                return 1;

            auto check_dims = [](const json &part, const std::string &circle_name)
            {
                if (!part[circle_name].contains("dims") || !part[circle_name]["dims"].contains("radius"))
                    return 1;
                if (!part[circle_name]["dims"]["radius"].is_number())
                    return 1;
                return 0;
            };

            if (check_dims(part_description, "smaller_circle") || check_dims(part_description, "bigger_circle"))
                return 1;
        }
        catch (const std::exception &err)
        {
            LOG_ERROR_STREAM("Some error occured: " << err.what());
            return 1;
        }
        return 0;
    }

    int FitBowl::setLabelData(const std::vector<Label> &labels)
    {
        for (auto label : labels)
        {
            if (label.fit_method == "bowl")
            {
                _raw_data = label.raw_data;
                return 0;
            }
        }
        LOG_ERROR_STREAM("Could not obtain information for plate fit method");
        return 1;
    }

} // namespace estimate_shape
