#include "fit_banana/fit_banana.hpp"

namespace estimate_shape
{
    FitBanana::FitBanana(estimate_shape::CameraParameters cam1_params, estimate_shape::CameraParameters cam2_params, std::vector<std::string> &args, json &default_parameters, bool debug)
        : IFitMethod(cam1_params, cam2_params, args, default_parameters, debug),
          _item_ptcld(new pcl::PointCloud<pcl::PointXYZ>),
          _obb_transform(Eigen::Matrix4f::Identity()),
          _item_ptcld_transformed(new pcl::PointCloud<pcl::PointXYZ>),
          _segment_len(0.06f)
    {
        if (args.size() != default_parameters.size())
        {
            try
            {
                _segment_len = default_parameters["segment_len"];
            }
            catch (const nlohmann::detail::type_error &e)
            {
                std::cerr << "There was some problem retrieving values from JSON parameters. Running with default for specified fitting method\n";
            }
        }
        else if (validateNumericParameter(args[0], _segment_len))
        {
            _segment_len = default_parameters["segment_len"];
        }
    }

    FitBanana::~FitBanana()
    {
    }

    bool FitBanana::_assignBananaElements(std::vector<ItemElement> &item_elements)
    {
        Timer timer("Saving results to DB structures...", _debug);

        json primitive_shapes;
        json positions;
        json orientations;
        for (size_t item_element_idx = 0; item_element_idx < _segments_no; item_element_idx++)
        {
            if (_segments_dims[item_element_idx][1] <= 0)
            {
                std::string segment_name = _element_label_name + std::to_string(item_element_idx);
                //if it is the first or last segment set radius for the closest one
                if (item_element_idx == 0)
                    primitive_shapes[segment_name]["dims"]["radius"] = _segments_dims[item_element_idx + 1][1];
                if (item_element_idx == 5)
                    primitive_shapes[segment_name]["dims"]["radius"] = _segments_dims[item_element_idx - 1][1];
                else //for the middle one use the neighbourhood
                    primitive_shapes[segment_name]["dims"]["radius"] = (_segments_dims[item_element_idx - 1][1] + _segments_dims[item_element_idx + 1][1]) / 2;
                primitive_shapes[segment_name]["dims"]["height"] = _segments_dims[item_element_idx][0];
                primitive_shapes[segment_name]["fit_method"] = _segments_fit_method[item_element_idx];
                positions[segment_name]["x"] = _segments_positions[item_element_idx][0];
                positions[segment_name]["y"] = _segments_positions[item_element_idx][1];
                positions[segment_name]["z"] = _segments_positions[item_element_idx][2];
                orientations[segment_name]["x"] = _segments_orientations[item_element_idx].x();
                orientations[segment_name]["y"] = _segments_orientations[item_element_idx].y();
                orientations[segment_name]["z"] = _segments_orientations[item_element_idx].z();
                orientations[segment_name]["w"] = _segments_orientations[item_element_idx].w();
            }
            else
            {
                std::string segment_name = _element_label_name + std::to_string(item_element_idx);
                primitive_shapes[segment_name]["dims"]["height"] = _segments_dims[item_element_idx][0];
                primitive_shapes[segment_name]["dims"]["radius"] = _segments_dims[item_element_idx][1];
                primitive_shapes[segment_name]["fit_method"] = _segments_fit_method[item_element_idx];
                positions[segment_name]["x"] = _segments_positions[item_element_idx][0];
                positions[segment_name]["y"] = _segments_positions[item_element_idx][1];
                positions[segment_name]["z"] = _segments_positions[item_element_idx][2];
                orientations[segment_name]["x"] = _segments_orientations[item_element_idx].x();
                orientations[segment_name]["y"] = _segments_orientations[item_element_idx].y();
                orientations[segment_name]["z"] = _segments_orientations[item_element_idx].z();
                orientations[segment_name]["w"] = _segments_orientations[item_element_idx].w();
            }
        }
        item_elements[0].primitive_shape = primitive_shapes;
        item_elements[0].position = positions;
        item_elements[0].orientation = orientations;
        return true;
    }

    int FitBanana::fit(std::vector<ItemElement> &item_elements)
    {
        Timer timer("FitBanana::fit", _debug);

        _item_ptcld->points.clear();
        _item_ptcld_segments_indices.clear();
        _segments_dims.clear();
        _segments_positions.clear();
        _segments_orientations.clear();
        // Accumulate all input point clouds in case user wants to use this fitting method for objects different than bananas.
        for (size_t idx = 0; idx < item_elements.size(); ++idx)
            *_item_ptcld += *item_elements[idx].pcl_merged;
        helpers::commons::statisticalOutlierRemovalFilter(_item_ptcld);
        _computeOBB(_item_ptcld, _obb_transform, _obb_rot, _obb_pos);
        pcl::transformPointCloud(*_item_ptcld, *_item_ptcld_transformed, _obb_transform);
        _dividePtcld(_item_ptcld_transformed, _item_ptcld_segments_indices);
        if (!_createCylinders(_item_ptcld_transformed, _segment_len, _obb_transform, _item_ptcld_segments_indices, _cylinders_coefficients))
        {
            _logger->log("FitBanana::fit: can not fit any cylinder to any segment");
            return 1;
        };
        _assignBananaElements(item_elements);

        if (false)
        {
            size_t segment_idx = 0;
            pcl::visualization::PCLVisualizer viewer;
            viewer.addPointCloud(_item_ptcld, "banana");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "banana");
            for (auto cylinder_coff : _cylinders_coefficients)
            {
                viewer.addCylinder(cylinder_coff, std::to_string(segment_idx));
                viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, ((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)), ((double)rand() / (RAND_MAX)), std::to_string(segment_idx));
                segment_idx++;
            }
            viewer.setRepresentationToWireframeForAllActors();
            viewer.addCoordinateSystem(1.0);
            viewer.spin();
        }
        return 0;
    }

    bool FitBanana::_computeOBB(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Matrix4f &obb_transform, Eigen::Quaternionf &obb_rot, Eigen::Vector3f &obb_pos)
    {
        Timer timer("Computing oriented bounding box...", _debug);
        // std::cout<<setw(70) << "Computing oriented bounding box... ";
        // std::cout<<left;

        // compute principal direction
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*obj_ptcld, centroid);
        Eigen::Matrix3f covariance;
        pcl::computeCovarianceMatrixNormalized(*obj_ptcld, centroid, covariance);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
        Eigen::Matrix3f eigDx = eigen_solver.eigenvectors();
        eigDx.col(2) = eigDx.col(0).cross(eigDx.col(1));

        // move the points to the that reference frame
        obb_transform.block<3, 3>(0, 0) = eigDx.transpose();
        obb_transform.block<3, 1>(0, 3) = -1.f * (obb_transform.block<3, 3>(0, 0) * centroid.head<3>());
        pcl::PointCloud<pcl::PointXYZ> cPoints;
        pcl::transformPointCloud(*obj_ptcld, cPoints, obb_transform);

        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(cPoints, min_pt, max_pt);
        const Eigen::Vector3f mean_diag = 0.5f * (max_pt.getVector3fMap() + min_pt.getVector3fMap());

        const Eigen::Quaternionf qfinal(eigDx);
        const Eigen::Vector3f tfinal = eigDx * mean_diag + centroid.head<3>();

        // draw the cloud and the box
        pcl::visualization::PCLVisualizer viewer;
        viewer.addPointCloud(obj_ptcld);
        viewer.addCube(tfinal, qfinal, max_pt.x - min_pt.x, max_pt.y - min_pt.y, max_pt.z - min_pt.z);
        viewer.spin();

        // final transform
        // obb_rot = eigDx;
        // obb_pos = eigDx * mean_diag + centroid.head<3>();
        // std::cout << " Done " << std::endl;

        return true;
    }

    bool FitBanana::_dividePtcld(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_transformed_ptcld, std::vector<pcl::Indices> &out_ptcld_segments_indices)
    {
        Timer timer("Dividing banana pointcloud to segments", _debug);

        pcl::PointXYZ whole_ptcld_min_pt, whole_ptcld_max_pt;
        // auto start = std::chrono::system_clock::now();
        // Eigen::Matrix3f covariance_matrix;
        // Eigen::Vector4f centroid;
        // Eigen::Vector3f centroid_3f = centroid.head<3>();
        // float threshold = 3;
        // // pcl::compute3DCentroid< pcl::PointXYZ>(*whole_transformed_ptcld,)
        // pcl::computeMeanAndCovarianceMatrix(*whole_transformed_ptcld, covariance_matrix, centroid);
        // float x_variance = std::sqrt(std::abs(covariance_matrix(0, 0)));
        // float y_variance = std::sqrt(std::abs(covariance_matrix(1, 1)));
        // float z_variance = std::sqrt(std::abs(covariance_matrix(2, 2)));
        // std::cout << "Pointcloud size input : " << whole_transformed_ptcld->points.size() << std::endl;
        // helpers::commons::passThroughFilter(whole_transformed_ptcld, "x", threshold * x_variance, threshold * x_variance);
        // std::cout << "Pointcloud size x : " << whole_transformed_ptcld->points.size() << std::endl;
        // helpers::commons::passThroughFilter(whole_transformed_ptcld, "y", threshold * y_variance, threshold * y_variance);
        //  std::cout << "Pointcloud size y : " << whole_transformed_ptcld->points.size() << std::endl;
        // helpers::commons::passThroughFilter(whole_transformed_ptcld, "z", threshold * z_variance, threshold * z_variance);
        //  std::cout << "Pointcloud size z : " << whole_transformed_ptcld->points.size() << std::endl;
        // // std::cout << "Segment Radius calculated with ransac is incorrect : " << cylinder_coefficients->values[6] << std::endl;
        // std::cout << "Calculating using covariance matrix and centroid information " << std::endl;
        // std::cout << "Variance for x values : " << x_variance << std::endl;
        // std::cout << "Variance for y values : " << y_variance << std::endl;
        // std::cout << "Variance for z values : " << z_variance << std::endl;
        // auto end = std::chrono::system_clock::now();
        // RCLCPP_ERROR(rclcpp::get_logger("banana"), "Elapsed time:   '%lld'  ms, for computing meand and covariance matrix ", std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

        // std::cout << "centroid position : " << centroid << std::endl;

        pcl::getMinMax3D(*whole_transformed_ptcld, whole_ptcld_min_pt, whole_ptcld_max_pt);
        _segments_no = 5;
        _segment_len = (whole_ptcld_max_pt.z - whole_ptcld_min_pt.z) / _segments_no;
        try
        {
            if (_segment_len <= 0 || _segments_no > MAX_NO_OF_SEGMENTS)
            {
                _logger->log("FitBanana::_dividePtcld:", "wrong segment lengh ");
                // throw std::runtime_error("FitBanana::_dividePtcld: wrong segment lengh");
            }
            // std::vector<std::string> segments_names;
            pcl::Indices segment_indices;
            for (size_t segment_idx = 0; segment_idx < _segments_no; segment_idx++)
            {
                out_ptcld_segments_indices.push_back(segment_indices);
                float segment_start_idx = whole_ptcld_min_pt.z + segment_idx * _segment_len;
                float segment_end_idx = whole_ptcld_min_pt.z + (segment_idx + 1) * _segment_len;
                _passThroughFilter(whole_transformed_ptcld, "z", segment_start_idx, segment_end_idx, false, out_ptcld_segments_indices[segment_idx]);
            }
        }
        catch (std::runtime_error &e)
        {
            std::cerr << e.what() << "\n";
        }

        return true;
    }
    bool FitBanana::_createCylinders(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_transformed_ptcld, float /*segment_len*/, Eigen::Matrix4f &obb_transform, std::vector<pcl::Indices> &ptcld_segments_indices, std::vector<pcl::ModelCoefficients> &out_cs_coefficients)
    {
        Timer timer("Estimating banana shape with cylinders", _debug);
        size_t element_idx = 0;
        for (auto ptcld_segment_indices : ptcld_segments_indices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr segment_ptcld(new pcl::PointCloud<pcl::PointXYZ>);
            Eigen::Affine3f segment_pose;
            std::vector<float> segment_dim;
            pcl::ModelCoefficients::Ptr c_coefficients(new pcl::ModelCoefficients);
            _extractPtcld(whole_transformed_ptcld, ptcld_segment_indices, segment_ptcld);
            // if (ptcld_segment_indices.size() <= 1)
            // {
            //     std::cout << "Segment Ptcld size is to small to perform segmentation, skipping segment" << std::endl;
            //     _segments_no--;
            //     return false;
            // }
            pcl::transformPointCloud(*segment_ptcld, *segment_ptcld, obb_transform.inverse());
            if (!_fitCylinder(segment_ptcld, _segment_len, c_coefficients, segment_pose, segment_dim))
            {
                _segments_no--;
                if (_segments_no > 0)
                    continue;
                else
                    return false;
            };
            out_cs_coefficients.push_back(*c_coefficients);
            // std::cout << "out_cs_coefficients" << out_cs_coefficients[0]<<std::endl;
            _segments_ptclds.push_back(segment_ptcld);
            _segments_positions.push_back(segment_pose.translation());
            // Eigen::Matrix3f obb_rot = obb_transform.block(0, 0, 3, 3);
            Eigen::Quaternionf segment_orientation(segment_pose.rotation());
            _segments_orientations.push_back(segment_orientation);
            _segments_dims.push_back(segment_dim);
            _segments_fit_method.push_back(_element_fit_method);
            element_idx++;
        }
        return true;
    }

    bool FitBanana::_extractPtcld(pcl::PointCloud<pcl::PointXYZ>::Ptr whole_ptcld, pcl::Indices &ptcld_segment_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr segment_ptcld)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointIndices::Ptr Inliers(new pcl::PointIndices);
        Inliers->indices = ptcld_segment_indices;
        extract.setInputCloud(whole_ptcld);
        extract.setIndices(Inliers);
        extract.setNegative(false);
        extract.filter(*segment_ptcld);
        return true;
    }

    int FitBanana::_passThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string axis, float min_limit, float max_limit, bool negative, pcl::Indices &out_cloud_indices)
    {
        if (cloud->points.size() == 0)
        {
            _logger->log("FitBanana::_passThroughFilter", "no points inside input pointcloud.");
            return 1;
        }
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName(axis);
        pass.setFilterLimits(min_limit, max_limit);
        pass.setFilterLimitsNegative(negative);
        pass.filter(out_cloud_indices);
        return 0;
    }

    bool FitBanana::_fitCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_ptcld, float segment_len, pcl::ModelCoefficients::Ptr coeffients_cylinder, Eigen::Affine3f &c_pose,
                                 std::vector<float> &c_dim)
    {
        Timer timer("FitBanana::_fitCylinder", _debug);
        // Create segmentation object for cylinder segmentation and set all the parameters
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

        //  Normals
        pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ>>(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr normalsFilter(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normalEstimator;
        normalEstimator.setSearchMethod(tree);
        normalEstimator.setInputCloud(cylinder_ptcld);
        normalEstimator.setKSearch(20);
        normalEstimator.setNumberOfThreads(std::thread::hardware_concurrency() / 2);
        normalEstimator.compute(*normalsFilter);
        pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
        try
        {
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_CYLINDER);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setNormalDistanceWeight(0.2);
            seg.setMaxIterations(1000);
            seg.setDistanceThreshold(0.05);
            // seg.setAxis(Eigen::Vector3f::UnitZ());
            // seg.setEpsAngle(M_PI/6); //30 degrees
            seg.setRadiusLimits(0, 0.05);
            seg.setInputCloud(cylinder_ptcld);
            seg.setInputNormals(normalsFilter);
            seg.setNumberOfThreads(std::thread::hardware_concurrency() / 2);
            seg.setProbability(0.4);
            // pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
            // seg.setInputCloud(cylinder_ptcld);
            // seg.setInputNormals(normalsFilter);
            // seg.setOptimizeCoefficients(true);
            // seg.setModelType(pcl::SACMODEL_CYLINDER);
            // seg.setMethodType(pcl::SAC_RANSAC);
            // //seg.setNormalDistanceWeight(0.1);
            // seg.setNormalDistanceWeight(0.2); // for coarse data
            // seg.setMaxIterations(1000);
            // seg.setDistanceThreshold(0.3);
            // seg.setRadiusLimits(0, 0.06); // radius is within 8 centimeters

            seg.segment(*inliers_cylinder, *coeffients_cylinder);
            if (coeffients_cylinder->values.size() == 0)
            {
                throw std::runtime_error("Segmentation couldnt be performed correctly");
            }
        }

        catch (const std::runtime_error &error)
        {

            std::cout << error.what() << '\n';
            return false;
        }
        // pcl::MedianFilter<pcl::PointCloud<pcl::PointXYZ>> dupa;
        // dupa.
        // Ouput extracted cylinder
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_ptcld_filtered (new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::ExtractIndices<pcl::PointXYZ> extract;
        // extract.setInputCloud(cylinder_ptcld);
        // extract.setIndices(inliers_cylinder);
        // extract.filter(*cylinder_ptcld_filtered);

        // float sum_D = 0.0;
        // float sum_Ave = 0.0;
        // float x0 = coeffients_cylinder->values[0];
        // float y0 = coeffients_cylinder->values[1];
        // float z0 = coeffients_cylinder->values[2];
        // float l = coeffients_cylinder->values[3];
        // float m = coeffients_cylinder->values[4];
        // float n = coeffients_cylinder->values[5];
        // float r0 = coeffients_cylinder->values[6];
        // for (int i = 0; i < cylinder_ptcld->points.size(); i++)
        // {
        //     float x = cylinder_ptcld->points[i].x;
        //     float y = cylinder_ptcld->points[i].y;
        //     float z = cylinder_ptcld->points[i].z;
        //     // D=part1+part2
        //     float part1 = pow(x - x0, 2) + pow(y - y0, 2) + pow(z - z0, 2) - pow(r0, 2);
        //     float part2 = -pow(l * (x - x0) + m * (y - y0) + n * (z - z0), 2) / (l * l + m * m + n * n);
        //     sum_D += pow(part1 + part2, 2);
        //     sum_Ave += fabs(part1 + part2);
        // }
        // std::cerr << "The average difference is " << sum_Ave / cylinder_ptcld->points.size() << std::endl;
        // ;
        // std::cerr << "The Difference of average difference is " << sum_D / cylinder_ptcld->points.size() << std::endl;
        // ;

        // evaluate the cylinder quation
        c_dim.resize(2);
        c_dim[0] = _computeCylinderHeight(coeffients_cylinder, cylinder_ptcld);
        c_dim[1] = (coeffients_cylinder->values[6]);
        pcl::PointXYZ center = _computeCylinderCentroid(coeffients_cylinder, cylinder_ptcld, c_dim);
        c_pose = Eigen::Translation3f(center.getVector3fMap()) * Eigen::Quaternionf::Identity();
        _computeCylinderRotation(coeffients_cylinder, c_pose);

        Eigen::Vector3f c_len(coeffients_cylinder->values[3], coeffients_cylinder->values[4], coeffients_cylinder->values[5]);

        Eigen::Vector3f c_shift_vector(0, 0, segment_len / 2);
        c_len = c_len * segment_len;
        coeffients_cylinder->values[3] = c_len[0];
        coeffients_cylinder->values[4] = c_len[1];
        coeffients_cylinder->values[5] = c_len[2];

        Eigen::Vector3f wc_shift_vector(c_pose.rotation() * c_shift_vector);
        coeffients_cylinder->values[0] = c_pose.translation()[0] - wc_shift_vector[0];
        coeffients_cylinder->values[1] = c_pose.translation()[1] - wc_shift_vector[1];
        coeffients_cylinder->values[2] = c_pose.translation()[2] - wc_shift_vector[2];

        Eigen::Quaternionf c_rot(c_pose.rotation());
        return true;
    }

    float FitBanana::_computeCylinderHeight(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld)
    {
        Eigen::Vector3f rotation_vector(cylinder_coefficients->values[3], cylinder_coefficients->values[4], cylinder_coefficients->values[5]);
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(rotation_vector, Eigen::Vector3f::UnitZ());

        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ rot_center(0, 0, 0);
        _rotatePtcldAroundPoint(obj_ptcld, rot, rot_center, rotated_cloud);

        float min = rotated_cloud->getMatrixXfMap().row(2).minCoeff();
        float max = rotated_cloud->getMatrixXfMap().row(2).maxCoeff();
        float cylinder_height = max - min;

        // if (cylinder_coefficients->values[6] <= 0)
        // {

        //     float min_x = rotated_cloud->getMatrixXfMap().row(0).minCoeff();
        //     float max_x = rotated_cloud->getMatrixXfMap().row(0).maxCoeff();
        //     float min_y = rotated_cloud->getMatrixXfMap().row(1).minCoeff();
        //     float max_y = rotated_cloud->getMatrixXfMap().row(1).maxCoeff();
        //     float min_z = rotated_cloud->getMatrixXfMap().row(2).minCoeff();
        //     float max_z = rotated_cloud->getMatrixXfMap().row(2).maxCoeff();
        //     float cylinder_x_axis = max_x - min_x;
        //     float cylinder_y_axis = max_y - min_y;
        //     float cylinder_z_axis = max_z - min_z;
        //     std::cout << "Segment Radius calculated with ransac is incorrect : " << cylinder_coefficients->values[6] << std::endl;
        //     cylinder_coefficients->values[6] = ((cylinder_x_axis + cylinder_y_axis + cylinder_z_axis) / 6);
        //     std::cout << "New radius value: " << cylinder_coefficients->values[6] << std::endl;
        //     // Eigen::Vector4f segment_centroid;
        // Eigen::Vector4f min_point,;
        // Eigen::Matrix3f covariance_matrix;
        // float x_variance, y_variance, xy_variance;
        // pcl::computeMeanAndCovarianceMatrix(*rotated_cloud, covariance_matrix, segment_centroid);
        // pcl::getMinMax3D()
        // x_variance = std::sqrt(std::abs(covariance_matrix(0, 0)));
        // y_variance = std::sqrt(std::abs(covariance_matrix(1, 1)));
        // xy_variance = std::sqrt(std::abs(covariance_matrix(0, 1)));
        // std::cout << "Segment Radius calculated with ransac is incorrect : " << cylinder_coefficients->values[6] << std::endl;
        // std::cout << "Calculating using covariance matrix and centroid information " << std::endl;
        // std::cout << "Variance for x values : " << x_variance << std::endl;
        // std::cout << "Variance for y values : " << y_variance << std::endl;
        // std::cout << "Variance for xy values : " << xy_variance << std::endl;
        // std::cout << "Segment Centroid : " << segment_centroid << std::endl;
        // cylinder_coefficients->values[6] = ((x_variance + y_variance) / 2) * 3;
        // std::cout << "New radius value: " << cylinder_coefficients->values[6] << std::endl;
        // }

        return cylinder_height;
    }
    pcl::PointXYZ FitBanana::_computeCylinderCentroid(pcl::ModelCoefficients::Ptr cylinder_coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, std::vector<float> &out_obj_dim)
    {

        Eigen::Vector3f rotation_vector(cylinder_coefficients->values[3], cylinder_coefficients->values[4], cylinder_coefficients->values[5]);
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(rotation_vector, Eigen::Vector3f::UnitZ());

        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ rot_center(0, 0, 0);
        _rotatePtcldAroundPoint(obj_ptcld, rot, rot_center, rotated_cloud);

        float min = rotated_cloud->getMatrixXfMap().row(2).minCoeff();

        pcl::PointXYZ center;
        Eigen::Vector3f center_vec(cylinder_coefficients->values[0], cylinder_coefficients->values[1], cylinder_coefficients->values[2]);
        center.getVector3fMap() = center_vec;
        center.getVector3fMap() = rot * center.getVector3fMap();
        center.z = min + (out_obj_dim[0] / 2);
        center.getVector3fMap() = rot.inverse() * center.getVector3fMap();
        return center;
    }

    int FitBanana::_computeCylinderRotation(pcl::ModelCoefficients::Ptr cylinder_coefficients, Eigen::Affine3f &out_obj_pose)
    {
        //fix xy axises so that x is parrarell to the table
        Eigen::Vector3f rotation_vector(cylinder_coefficients->values[3], cylinder_coefficients->values[4], cylinder_coefficients->values[5]);
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(rotation_vector, Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf x_rot;
        Eigen::Vector3f z_axis = rot.toRotationMatrix().col(2);
        Eigen::Vector3f z_axis_projected = z_axis;
        //project z axis on table
        z_axis_projected(2) = 0;
        x_rot.setFromTwoVectors(z_axis_projected, rotation_vector);
        rot = x_rot * rot;
        Eigen::Vector3f y_axis = z_axis_projected;
        // construct 90 degrees (around z) rotation matrix
        Eigen::Matrix3f z_rot = helpers::commons::assignRotationMatrixAroundZ(M_PI / 2);
        y_axis = z_rot * y_axis;
        // in flattend rot - y axis is juz unit vector
        Eigen::Vector3f x_axis(0, 0, -1);
        //normalizeCommons
        x_axis.normalize();
        y_axis.normalize();
        z_axis_projected.normalize();
        // construct projected rotation matrix
        Eigen::Matrix3f final_rot;
        final_rot.col(0) = x_axis;
        final_rot.col(1) = y_axis;
        final_rot.col(2) = z_axis_projected;
        Eigen::Quaternionf final_rot_quat(final_rot);
        // rotate projected rot by angle between table and initial z axis
        final_rot_quat = x_rot * final_rot_quat;
        final_rot_quat = final_rot_quat * z_rot;
        out_obj_pose = Eigen::Translation3f(out_obj_pose.translation()) * final_rot_quat;
        return 0;
    }

    int FitBanana::_rotatePtcldAroundPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld, Eigen::Quaternionf rot, pcl::PointXYZ rot_center, pcl::PointCloud<pcl::PointXYZ>::Ptr out_ptcld)
    {
        out_ptcld->points.resize(obj_ptcld->points.size());
        for (auto it = obj_ptcld->points.begin(); it != obj_ptcld->points.end(); it++)
        {
            out_ptcld->points[it - obj_ptcld->points.begin()].getVector3fMap() = (rot * (it->getVector3fMap() - rot_center.getVector3fMap())) + rot_center.getVector3fMap();
        }
        return 0;
    }
} // namespace estimate_shape
