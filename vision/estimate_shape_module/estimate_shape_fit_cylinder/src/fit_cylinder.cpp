#include "estimate_shape_fit_cylinder/fit_cylinder.hpp"

namespace estimate_shape
{
    FitCylinder::FitCylinder(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters),
          _radius_min(0.03),
          _radius_max(0.05)
    {
        if (args.size() != default_parameters.size())
        {
            try
            {
                _radius_min = default_parameters["radius_min"];
                _radius_max = default_parameters["radius_max"];
            }
            catch (const nlohmann::detail::type_error &e)
            {
                std::cerr << "There was some problem retrieving values from JSON parameters. Running with default for specified fitting method\n";
            }
        }
        else
        {
            int radius_min_success = validateNumericParameter(args[0], _radius_min);
            int radius_max_success = validateNumericParameter(args[1], _radius_max);
            if (radius_min_success || radius_max_success)
            {
                _radius_min = default_parameters["radius_min"];
                _radius_max = default_parameters["radius_max"];
            }
        }
    }

    FitCylinder::~FitCylinder()
    {
    }

    int FitCylinder::fit(Item &item)
    {
        Timer timer("FitCylinder::fit", LOGGER);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_ptcld1_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_ptcld2_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld2(new pcl::PointCloud<pcl::PointXYZ>);
        *obj_ptcld1_rgb += *item.item_elements[0].element_pcl_1;
        *obj_ptcld2_rgb += *item.item_elements[0].element_pcl_2;
        pcl::copyPointCloud(*obj_ptcld1_rgb, *obj_ptcld1);
        pcl::copyPointCloud(*obj_ptcld2_rgb, *obj_ptcld2);
        // helpers::commons::statisticalOutlierRemovalFilter(obj_ptcld1);
        // helpers::commons::statisticalOutlierRemovalFilter(obj_ptcld2);

        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::ModelCoefficients::Ptr cylinder_coefficients(new pcl::ModelCoefficients);

        _estimateNormals(obj_ptcld1, CamerasFrames::camera_1, cloud_with_normals1);
        _estimateNormals(obj_ptcld2, CamerasFrames::camera_2, cloud_with_normals2);

        *cloud_with_normals = *cloud_with_normals1 + *cloud_with_normals2;

        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld(new pcl::PointCloud<pcl::PointXYZ>);
        *obj_ptcld = *obj_ptcld1 + *obj_ptcld2;
        _fitModelNormals(cloud_with_normals, pcl::SACMODEL_CYLINDER, *cylinder_coefficients);
        if (cylinder_coefficients->values.size() == 0)
        {
            return 1;
        }

        std::vector<float> out_obj_dim;
        out_obj_dim.resize(2);
        out_obj_dim[0] = _computeCylinderHeight(cylinder_coefficients, obj_ptcld);

        //cylinder radius
        out_obj_dim[1] = (cylinder_coefficients->values[6]);
        pcl::PointXYZ center = _computeCylinderCentroid(cylinder_coefficients, obj_ptcld, out_obj_dim);

        Eigen::Affine3f out_obj_pose;
        out_obj_pose = Eigen::Translation3f(center.getVector3fMap()) * Eigen::Quaternionf::Identity();
        // out_obj_pose.position = commons::fromEigenToRosPosition(center.getVector3fMap());
        _computeCylinderRotation(cylinder_coefficients, out_obj_pose);

        float radius;
        _computeCylinderRadius(center, obj_ptcld, out_obj_pose, radius);

        // Assign output
        item.pose = out_obj_pose;

        json part_description;
        part_description["fit_method"] = "cylinder";
        part_description["spawn_method"] = "cylinder";
        part_description["part_name"] = item.label;
        part_description["dims"]["height"] = out_obj_dim[0];
        part_description["dims"]["radius"] = radius;

        json position;
        position["x"] = 0.0;
        position["y"] = 0.0;
        position["z"] = 0.0;
        part_description["pose"]["position"] = position;

        json orientation;
        orientation["x"] = 0.0;
        orientation["y"] = 0.0;
        orientation["z"] = 0.0;
        orientation["w"] = 1.0;
        part_description["pose"]["orientation"] = orientation;

        item.item_elements[0].parts_description = {part_description};
        return 0;
    }

    int FitCylinder::_computeCylinderRadius(pcl::PointXYZ &center, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Affine3f &out_obj_pose, float &out_radius)
    {
        Eigen::Vector4f line_point(center.x, center.y, center.z, 0);
        Eigen::Vector4f line_dir = out_obj_pose.matrix().col(2);
        std::vector<double> radius_vector;
        for (auto &pt : obj_ptcld->points)
        {
            Eigen::Vector4f point(pt.x, pt.y, pt.z, 0);
            radius_vector.push_back(sqrt(pcl::sqrPointToLineDistance(point, line_point, line_dir)));
        }
        // double radius = *std::max_element(radius_vector.begin(), radius_vector.end());
        constexpr float nth = 0.9;
        size_t n = radius_vector.size() * nth;
        std::nth_element(radius_vector.begin(), radius_vector.begin() + n, radius_vector.end());
        double radius_median = radius_vector[n];
        if (static_cast<size_t>(std::floor(radius_vector.size() * nth)) % 2 == 1)
        {
            out_radius = static_cast<float>(radius_median);
        }
        else
        {
            // std::nth_element(radius_vector.begin(), radius_vector.begin() + n - 1, radius_vector.end());
            out_radius = static_cast<float>(0.5 * (radius_median + radius_vector[n - 1]));
            // std::cout<<"radius"<<radius<<std::endl;
            // std::cout<<"radius-median"<<out_radius<<std::endl;
        }
        return 0;
    }

    int FitCylinder::_estimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, std::string frame, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr out_cloud_with_normals)
    {
        int k_neighbours = 20;
        if (!helpers::vision::checkPointsAmmount(obj_ptcld, k_neighbours))
        {
            LOG_DEBUG_STREAM("FitCylinder::_estimateNormals: Not enough points in pointcloud; nr of points: " << obj_ptcld->points.size());
            return 1;
        }

        //estimationg the normals
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNumberOfThreads(std::thread::hardware_concurrency() / 2);
        ne.setInputCloud(obj_ptcld);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        ne.setKSearch(k_neighbours);
        //ne.setRadiusSearch (0.04);
        // Eigen::Affine3f camera_pose = obtainCameraPosition(frame);
        Eigen::Affine3f camera_pose = _camera_poses[frame];

        ne.setViewPoint(camera_pose.translation().x(), camera_pose.translation().y(), camera_pose.translation().z());
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        ne.compute(*cloud_normals);
        pcl::concatenateFields(*obj_ptcld, *cloud_normals, *out_cloud_with_normals);
        return 0;
    }

    void FitCylinder::_fitModelNormals(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals, uint8_t model_type, pcl::ModelCoefficients &out_model_coefficients)
    {
        if (cloud_with_normals->size() == 0)
        {
            LOG_DEBUG("FitCylinder::_fitModelNormals: No points in pointcloud with normals");
            return;
        }

        pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> segmentation;
        pcl::PointIndices model_inlierIndices;

        segmentation.setInputCloud(cloud_with_normals);
        segmentation.setInputNormals(cloud_with_normals);
        segmentation.setModelType(model_type);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setNormalDistanceWeight(0.1);
        segmentation.setDistanceThreshold(0.03); //was 0.05
        // segmentation.setOptimizeCoefficients(true);
        segmentation.setRadiusLimits(_radius_min, _radius_max);
        segmentation.setMaxIterations(150);
        segmentation.setProbability(0.95); // was 0.4
        segmentation.segment(model_inlierIndices, out_model_coefficients);
    }

    float FitCylinder::_computeCylinderHeight(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld)
    {
        Eigen::Vector3f rotation_vector(cylinder_coefficients->values[3], cylinder_coefficients->values[4], cylinder_coefficients->values[5]);
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(rotation_vector, Eigen::Vector3f::UnitZ());

        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ rot_center(0, 0, 0);
        helpers::vision::rotatePtcldAroundPoint(obj_ptcld, rot, rot_center, rotated_cloud);
        // Eigen::Matrix3f covariance_matrix;
        // pcl::computeCovarianceMatrix(*rotated_cloud, covariance_matrix);
        // std::cout << "Eigen Matrix : " << covariance_matrix << std::endl;
        float min = rotated_cloud->getMatrixXfMap().row(2).minCoeff();
        float max = rotated_cloud->getMatrixXfMap().row(2).maxCoeff();
        float cylinder_height = max - min;
        return cylinder_height;
    }

    pcl::PointXYZ FitCylinder::_computeCylinderCentroid(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, std::vector<float> &out_obj_dim)
    {
        // compute cylinder height
        Eigen::Vector3f rotation_vector(cylinder_coefficients->values[3], cylinder_coefficients->values[4], cylinder_coefficients->values[5]);
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(rotation_vector, Eigen::Vector3f::UnitZ());

        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ rot_center(0, 0, 0);
        helpers::vision::rotatePtcldAroundPoint(obj_ptcld, rot, rot_center, rotated_cloud);

        float max = rotated_cloud->getMatrixXfMap().row(2).maxCoeff();

        pcl::PointXYZ center;
        Eigen::Vector3f center_vec(cylinder_coefficients->values[0], cylinder_coefficients->values[1], cylinder_coefficients->values[2]);
        center.getVector3fMap() = center_vec;
        center.getVector3fMap() = rot * center.getVector3fMap();
        center.z = max - (out_obj_dim[0] / 2);
        center.getVector3fMap() = rot.inverse() * center.getVector3fMap();
        return center;
    }

    int FitCylinder::_computeCylinderRotation(pcl::ModelCoefficients::Ptr cylinder_coefficients, Eigen::Affine3f &out_obj_pose)
    {
        //fix xy axises so that x is parrarell to the table
        Eigen::Vector3f rotation_vector(cylinder_coefficients->values[3], cylinder_coefficients->values[4], cylinder_coefficients->values[5]);
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), rotation_vector);

        out_obj_pose = Eigen::Translation3f(out_obj_pose.translation()) * rot;
        return 0;
    }
} // namespace estimate_shape
