#include "estimate_shape_fit_sphere/fit_sphere.hpp"

namespace estimate_shape
{
    FitSphere::FitSphere(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
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

    FitSphere::~FitSphere()
    {
    }

    int FitSphere::fit(Item &item)
    {
        Timer timer("FitSphere::fit", LOGGER);

        pcl::PointCloud<pcl::PointXYZ>::Ptr obj_ptcld(new pcl::PointCloud<pcl::PointXYZ>);
        *obj_ptcld = *item.item_elements[0].pcl_merged;
        helpers::vision::statisticalOutlierRemovalFilter(obj_ptcld);

        pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);
        _fitModel(obj_ptcld, pcl::SACMODEL_SPHERE, *sphere_coefficients);

        if (sphere_coefficients->values.size() == 0)
        {
            LOG_DEBUG("FitSphere::fit: phere_coefficients->values.size() == 0");
            return 1;
        }

        std::vector<float> out_obj_dim;
        out_obj_dim.push_back(sphere_coefficients->values[3]);
        Eigen::Vector3f sphere_pose_vec(sphere_coefficients->values[0], sphere_coefficients->values[1], sphere_coefficients->values[2]);

        Eigen::Quaternionf rotation(1, 0, 0, 0);
        Eigen::Quaternionf x_angle;
        sphere_pose_vec[2] = 0.0;
        x_angle.setFromTwoVectors(Eigen::Vector3f::UnitX(), sphere_pose_vec);

        Eigen::Quaternionf final_rotation;
        final_rotation = x_angle * helpers::vision::assignRotationMatrixAroundY(-M_PI / 2);
        //rotate 90 degrees around z so that y axis is pointing upwards (agreement)
        final_rotation = final_rotation * helpers::vision::assignRotationMatrixAroundZ(M_PI / 2);
        Eigen::Affine3f out_obj_pose;
        out_obj_pose = Eigen::Translation3f(sphere_coefficients->values[0], sphere_coefficients->values[1], sphere_coefficients->values[2]) * final_rotation;

        // Assigning output info
        item.pose = out_obj_pose;

        json part_description;
        part_description["fit_method"] = "sphere";
        part_description["spawn_method"] = "sphere";
        part_description["part_name"] = item.label;
        part_description["dims"]["radius"] = out_obj_dim[0];

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

    int FitSphere::_fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, uint8_t model_type, pcl::ModelCoefficients &out_model_coefficients)
    {
        if (cloud->points.size() == 0)
        {
            LOG_DEBUG("FitSphere::_fitModel: No points in input pointcloud");
            return 1;
        }

        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setNumberOfThreads(std::thread::hardware_concurrency() / 2);
        pcl::PointIndices model_inlierIndices;
        segmentation.setInputCloud(cloud);
        segmentation.setModelType(model_type);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setDistanceThreshold(0.002);
        segmentation.setOptimizeCoefficients(false);
        segmentation.setRadiusLimits(0.01, 0.1);
        segmentation.setEpsAngle(90 / (180 / 3.141592654));
        segmentation.setMaxIterations(100000);
        segmentation.segment(model_inlierIndices, out_model_coefficients);
        return 0;
    }

} // namespace estimate_shape
