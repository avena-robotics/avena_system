#include "estimate_shape_fit_plate/fit_plate.hpp"

namespace estimate_shape
{
    FitPlate::FitPlate(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters),
          _item_ptcld_transformed(new pcl::PointCloud<pcl::PointXYZ>),
          _item_ptcld(new pcl::PointCloud<pcl::PointXYZ>),
          _obb_transform(Eigen::Matrix4f::Identity()),
          _show(false),
          _raw_data{}
    {
    }

    FitPlate::~FitPlate()
    {
    }

    int FitPlate::fit(Item &item)
    {
        Timer timer("FitPlate::fit", LOGGER);
        _clear();
        // std::vector<ItemElement> &item_elements = item.item_elements;
        _item_ptcld = item.item_elements[0].pcl_merged;
        LOG_INFO_STREAM("Loading dish pointcloud from database with item_id " << item.item_elements[0].item_id);

        float sphere_radius;
        float bigger_radius = _raw_data["item_description"]["upper_disc_radius"].get<float>();
        float smaller_radius = _raw_data["item_description"]["lower_disc_radius"].get<float>();
        float height = _raw_data["item_description"]["upper_disc_height"].get<float>();

        _computeSphereRadius(bigger_radius, smaller_radius, height, sphere_radius);
        std::cout << "Calculated sphere radius for plate is equal to : " << sphere_radius << std::endl;
        if (_computeOBB(_item_ptcld, _obb_transform))
            return 1;
        pcl::transformPointCloud(*_item_ptcld, *_item_ptcld_transformed, _obb_transform.inverse());
        Eigen::Vector4f centroid;
        if (_filterPointcloud(_item_ptcld_transformed, centroid))
            return 1;
        pcl::ModelCoefficients coeffients;
        if (_calculateSphereParameters(_item_ptcld_transformed, coeffients, sphere_radius * 0.9, sphere_radius * 1.1))
            return 1;
        if (_computeAndSavePlateData(item, _item_ptcld_transformed, coeffients))
            return 1;
        return 0;
    }

    void FitPlate::_clear()
    {
        _item_ptcld->points.clear();
        _item_ptcld_transformed->points.clear();
    }

    int FitPlate::_computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Matrix4f &obb_transform)
    {
        LOG_DEBUG("FitPlate::_computeOBB: Computing oriented bounding box...");
        // pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld_good(new pcl::PointCloud<pcl::PointXYZ>);
        // {
        //     helpers::Timer timer("feature extraction ", true);
        //     pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        //     if (obj_ptcld->points.size() == 0)
        //     {
        //         LOG_WARN("FitPlate::_computeOBB: no points to compute oriented bounding box");
        //         return 1;
        //     }
        //     feature_extractor.setInputCloud(obj_ptcld);
        //     feature_extractor.compute();
        //     pcl::PointXYZ min_point_OBB;
        //     pcl::PointXYZ max_point_OBB;
        //     pcl::PointXYZ position_OBB;
        //     Eigen::Matrix3f rotational_matrix_OBB;
        //     feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        //     obb_transform.block<3, 3>(0, 0) = rotational_matrix_OBB;
        //     Eigen::Vector3f obb_pos(position_OBB.getVector3fMap());
        //     obb_transform.block<3, 1>(0, 3) = obb_pos;

        //     *obj_ptcld_good = *obj_ptcld;
        // }
        // pcl::transformPointCloud(*obj_ptcld, *obj_ptcld_good, obb_transform.inverse());

        helpers::Timer timer("SelfAdjointEigenSolver ", LOGGER);
        if (obj_ptcld->points.size() == 0)
        {
            LOG_WARN("FitPlate::_computeOBB: no points to compute oriented bounding box");
            return 1;
        }

        // Compute principal directions
        Eigen::Vector4f pcaCentroid;
        pcl::compute3DCentroid(*obj_ptcld, pcaCentroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*obj_ptcld, pcaCentroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
        eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
                                                                                       ///    the signs are different and the box doesn't get correctly oriented in some cases.
        // Transform the original cloud to the origin where the principal components correspond to the axes.
        Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
        projectionTransform.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();
        projectionTransform.block<3, 1>(0, 3) = -1.f * (projectionTransform.block<3, 3>(0, 0) * pcaCentroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cPoints(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f projection = Eigen::Affine3f(projectionTransform.inverse());
        Eigen::Quaternionf rotation(projection.rotation());
        Eigen::Vector3f x_axis = rotation.matrix().col(0);
        Eigen::Vector3f z_axis = rotation.matrix().col(2);
        Eigen::Quaternionf rot_xz;
        rot_xz.setFromTwoVectors(z_axis, x_axis);
        rotation = rot_xz * rotation;
        projection = Eigen::Translation3f(projection.translation().x(), projection.translation().y(), projection.translation().z()) * rotation;
        obb_transform = projection.matrix();
        // pcl::transformPointCloud(*obj_ptcld, *cPoints, projection.inverse());
        return 0;
    }
    int FitPlate::_filterPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed, Eigen::Vector4f &centroid, float threshold)
    {
        if (item_ptcld_transformed->points.size() == 0)
        {
            LOG_WARN("FitPlate::_computeOBB: no points to compute oriented bounding box");
            return 1;
        }
        Eigen::Matrix3f covariance_matrix;
        threshold = 3;
        pcl::computeMeanAndCovarianceMatrix(*item_ptcld_transformed, covariance_matrix, centroid);
        float x_variance = std::sqrt(std::abs(covariance_matrix(0, 0)));
        float y_variance = std::sqrt(std::abs(covariance_matrix(1, 1)));
        float z_variance = std::sqrt(std::abs(covariance_matrix(2, 2)));
        helpers::vision::passThroughFilter(item_ptcld_transformed, "x", centroid(0) - threshold * x_variance, centroid(0) + threshold * x_variance);
        helpers::vision::passThroughFilter(item_ptcld_transformed, "y", centroid(1) - threshold * y_variance, centroid(1) + threshold * y_variance);
        helpers::vision::passThroughFilter(item_ptcld_transformed, "z", centroid(2) - threshold * z_variance, centroid(2) + threshold * z_variance);
        return 0;
    }

    int FitPlate::_calculateSphereParameters(pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed, pcl::ModelCoefficients &coeffients, float radius_min, float radius_max)
    {
        pcl::PointIndices inliers_cylinder;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        try
        {
            seg.setModelType(pcl::SacModel::SACMODEL_SPHERE);
            seg.setMethodType(pcl::SAC_PROSAC);
            seg.setNumberOfThreads(std::thread::hardware_concurrency() / 2);
            seg.setInputCloud(item_ptcld_transformed);
            seg.setDistanceThreshold(0.005);
            seg.setOptimizeCoefficients(true);
            seg.setRadiusLimits(radius_min, radius_max);
            seg.setMaxIterations(1000); // 100 000
            seg.segment(inliers_cylinder, coeffients);
        }
        catch (const std::exception &e)
        {
            LOG_WARN_STREAM("FitPlate::_computeOBB: no points to compute oriented bounding box. Error: " << e.what());
            return 1;
        }
        return 0;
    }
    int FitPlate::_computeAndSavePlateData(Item &item, pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_transformed, pcl::ModelCoefficients coeffients)
    {
        if (_item_ptcld_transformed->points.size() == 0)
        {
            LOG_WARN("FitPlate::_computePlateHeight: transformPointCloud was wrong");
            return false;
        }
        // auto start = std::chrono::system_clock::now();
        float min_z = item_ptcld_transformed->getMatrixXfMap().row(2).minCoeff();
        float max_z = item_ptcld_transformed->getMatrixXfMap().row(2).maxCoeff();
        // float out_height = abs(max_z - min_z);
        float out_height = _raw_data["item_description"]["upper_disc_height"].get<float>();
        Eigen::Vector3f obb_pos = _obb_transform.block<3, 1>(0, 3);
        Eigen::Matrix3f rotational_matrix_OBB = _obb_transform.block<3, 3>(0, 0);
        Eigen::Vector3f smaller_circle_position(0, 0, min_z); //lower circle position to pcl point
        smaller_circle_position = rotational_matrix_OBB * smaller_circle_position + obb_pos;
        pcl::PointXYZ smaller_pcl_pos(smaller_circle_position(0), smaller_circle_position(1), smaller_circle_position(2));

        Eigen::Vector3f bigger_circle_position(0, 0, max_z); //upper circle position to pcl point
        bigger_circle_position = rotational_matrix_OBB * bigger_circle_position + obb_pos;
        pcl::PointXYZ bigger_pcl_pos(bigger_circle_position(0), bigger_circle_position(1), bigger_circle_position(2));
        Eigen::Vector3f sphere_center_init(coeffients.values[0], coeffients.values[1], coeffients.values[2]);
        Eigen::Vector3f sphere_center_eigen = rotational_matrix_OBB * sphere_center_init + obb_pos;
        coeffients.values[0] = sphere_center_eigen.x();
        coeffients.values[1] = sphere_center_eigen.y();
        coeffients.values[2] = sphere_center_eigen.z();
        pcl::PointXYZ sphere_center(coeffients.values[0], coeffients.values[1], coeffients.values[2]);
        Eigen::Vector3f plate_vector(coeffients.values[0] - smaller_circle_position(0), coeffients.values[1] - smaller_circle_position(1), coeffients.values[2] - smaller_circle_position(2));
        Eigen::Quaternionf plate_orientation;
        plate_orientation.setFromTwoVectors(Eigen::Vector3f::UnitZ(), plate_vector);

        float bigger_radius_ = _raw_data["item_description"]["upper_disc_radius"].get<float>();
        float smaller_radius_ = _raw_data["item_description"]["lower_disc_radius"].get<float>();
        float out_plate_angle;
        float distance_smaller = std::sqrt(pcl::squaredEuclideanDistance(sphere_center, smaller_pcl_pos));
        float distance_bigger = std::sqrt(pcl::squaredEuclideanDistance(sphere_center, bigger_pcl_pos));
        if (distance_smaller >= distance_bigger)
        {
            LOG_DEBUG("Plate is standing correctly");
            bigger_radius_ = _plate_bigger_radius;
            smaller_radius_ = _plate_smaller_radius;
            float radius_diff = abs(bigger_radius_ - smaller_radius_);
            out_plate_angle = atan(radius_diff / out_height);
        }
        else
        {
            LOG_DEBUG("Plate is inverted");
            std::swap(smaller_circle_position, bigger_circle_position);
            std::swap(smaller_pcl_pos, bigger_pcl_pos);
            bigger_radius_ = _plate_bigger_radius;
            smaller_radius_ = _plate_smaller_radius;
            float radius_diff = abs(bigger_radius_ - smaller_radius_);
            out_plate_angle = atan(radius_diff / out_height);
            Eigen::Matrix3f orientation_matrix = plate_orientation.matrix();
            orientation_matrix *helpers::vision::assignRotationMatrixAroundX(180);
            orientation_matrix *helpers::vision::assignRotationMatrixAroundY(180);
            plate_orientation = orientation_matrix;
        }

        // Assign plate data
        Eigen::Translation3f plate_position((smaller_circle_position.x() + bigger_circle_position.x()) / 2,
                                            (smaller_circle_position.y() + bigger_circle_position.y()) / 2,
                                            (smaller_circle_position.z() + bigger_circle_position.z()) / 2);
        item.pose = plate_position * plate_orientation;

        json part_description;
        part_description["fit_method"] = "plate";
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
        json primitive_shape_ramp;
        primitive_shape_ramp["dims"]["height"] = out_height;
        primitive_shape_ramp["dims"]["angle"] = out_plate_angle;
        primitive_shape_ramp["fit_method"] = "ramp";
        part_description["ramp"] = primitive_shape_ramp;

        json position_smaller_circle = helpers::vision::assignJsonFromPosition(smaller_circle_position);
        json orientation_smaller_circle = helpers::vision::assignJsonFromQuaternion(plate_orientation);
        part_description["smaller_circle"]["dims"]["radius"] = smaller_radius_;
        part_description["smaller_circle"]["fit_method"] = "circle";
        part_description["smaller_circle"]["position"] = position_smaller_circle;
        part_description["smaller_circle"]["orientation"] = orientation_smaller_circle;

        json position_bigger_circle = helpers::vision::assignJsonFromPosition(bigger_circle_position);
        json orientation_bigger_circle = helpers::vision::assignJsonFromQuaternion(plate_orientation);
        part_description["bigger_circle"]["dims"]["radius"] = bigger_radius_;
        part_description["bigger_circle"]["fit_method"] = "circle";
        part_description["bigger_circle"]["position"] = position_bigger_circle;
        part_description["bigger_circle"]["orientation"] = orientation_bigger_circle;

        // auto end = std::chrono::system_clock::now();
        // RCLCPP_WARN(rclcpp::get_logger("fit_plate"), "Elapsed time:   '%lld'  ms, for computing plate dimensions ", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        if (_show)
        {
            pcl::visualization::PCLVisualizer viewer;
            viewer.addPointCloud(_item_ptcld, "plate");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "plate");
            viewer.addLine(smaller_pcl_pos, bigger_pcl_pos, "arrow");
            viewer.addLine(bigger_pcl_pos, sphere_center, "arrow2");
            // viewer.addPointCloudNormals(normalsFilter,"cloud_normals",0);
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "arrow");
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 15, "arrow2");
            viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "arrow2");
            viewer.addSphere(coeffients, "sphere");
            viewer.setRepresentationToWireframeForAllActors();
            viewer.addCoordinateSystem(0.1);
            viewer.spin();
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        item.item_elements[0].parts_description = {part_description};
        return 0;
    }

    int FitPlate::_computeSphereRadius(float big_radius, float smaller_radius, float height, float &sphere_radius)
    {
        sphere_radius = std::sqrt(big_radius * big_radius + std::pow((big_radius * big_radius - smaller_radius * smaller_radius - height * height) / (2 * height), 2));
        return 0;
    }

    int FitPlate::checkEstimation(Item &item)
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

    int FitPlate::setLabelData(const std::vector<Label> &labels)
    {
        for (auto label : labels)
        {
            if (label.fit_method == "plate")
            {
                _raw_data = label.raw_data;
                return 0;
            }
        }
        LOG_ERROR_STREAM("Could not obtain information for plate fit method");
        return 1;
    }

} // namespace estimate_shape
