#include "fit_broccoli/fit_broccoli.hpp"

namespace estimate_shape
{
    FitBroccoli::FitBroccoli(CameraParameters cam1_params, CameraParameters cam2_params, std::vector<std::string> &args, json &default_parameters, bool debug)
        : IFitMethod(cam1_params, cam2_params, args, default_parameters, debug), _radius_min(0.01f), _radius_max(0.04f), _voxel_size(0.005f)
    {
        if (args.size() != default_parameters.size())
        {
            try
            {
                _radius_min = default_parameters["radius_min"];
                _radius_max = default_parameters["radius_max"];
                _voxel_size = default_parameters["voxel_size"];
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
            int voxel_size_success = validateNumericParameter(args[2], _voxel_size);
            if (radius_min_success || radius_max_success || voxel_size_success)
            {
                _radius_min = default_parameters["radius_min"];
                _radius_max = default_parameters["radius_max"];
                _voxel_size = default_parameters["voxel_size"];
            }
        }
    }

    FitBroccoli::~FitBroccoli()
    {
    }

    // int FitBroccoli::init(std::vector<std::string> &args, json &default_parameters)
    // {
    //     if (args.size() != default_parameters.size())
    //     {
    //         try
    //         {
    //             _radius_min = default_parameters["radius_min"];
    //             _radius_max = default_parameters["radius_max"];
    //             _voxel_size = default_parameters["voxel_size"];
    //         }
    //         catch (const nlohmann::detail::type_error &e)
    //         {
    //             std::cerr << "There was some problem retrieving values from JSON parameters. Running with default for specified fitting method\n";
    //             return 1;
    //         }
    //     }
    //     else
    //     {
    //         int radius_min_success = validateNumericParameter(args[0], _radius_min);
    //         int radius_max_success = validateNumericParameter(args[1], _radius_max);
    //         int voxel_size_success = validateNumericParameter(args[2], _voxel_size);
    //         if (radius_min_success || radius_max_success || voxel_size_success)
    //         {
    //             _radius_min = default_parameters["radius_min"];
    //             _radius_max = default_parameters["radius_max"];
    //             _voxel_size = default_parameters["voxel_size"];
    //         }
    //     }
    //     return 0;
    // }

    int FitBroccoli::_sortBroccoliParts(std::vector<ItemElement> &item_elements)
    {
        std::map<std::string, int> broccoli_order;
        broccoli_order["broccoli_handle"] = 0;
        broccoli_order["broccoli_head"] = 1;

        std::sort(item_elements.begin(), item_elements.end(), [&broccoli_order](ItemElement &a, ItemElement &b) {
            return (broccoli_order[a.element_label] < broccoli_order[b.element_label]);
        });

        return 0;
    }

    int FitBroccoli::_filterInputPointclouds(std::vector<ItemElement> &item_elements, pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_head, pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_leg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_leg_cam1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(item_elements[0].element_pcl_1), *broccoli_leg_cam1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_leg_cam2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(item_elements[0].element_pcl_2), *broccoli_leg_cam2);

        // helpers::converters::streamToPointcloud(item_elements[0].element_pcl_1, *broccoli_leg_cam1);
        // helpers::converters::streamToPointcloud(item_elements[0].element_pcl_2, *broccoli_leg_cam2);
        helpers::commons::statisticalOutlierRemovalFilter(broccoli_leg_cam1);
        helpers::commons::statisticalOutlierRemovalFilter(broccoli_leg_cam2);
        if (!broccoli_leg)
            broccoli_leg = helpers::commons::makeSharedPcl<pcl::PointXYZ>();
        *broccoli_leg = *broccoli_leg_cam1 + *broccoli_leg_cam2;
        // helpers::converters::pointcloudToStream(*broccoli_leg, item_elements[0].pcl_merged);

        pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_head_cam1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(item_elements[1].element_pcl_1), *broccoli_head_cam1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_head_cam2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*(item_elements[1].element_pcl_2), *broccoli_head_cam2);

        // helpers::converters::streamToPointcloud(item_elements[1].element_pcl_1, *broccoli_head_cam1);
        // helpers::converters::streamToPointcloud(item_elements[1].element_pcl_2, *broccoli_head_cam2);
        helpers::commons::statisticalOutlierRemovalFilter(broccoli_head_cam1);
        helpers::commons::statisticalOutlierRemovalFilter(broccoli_head_cam2);
        if (!broccoli_head)
            broccoli_head = helpers::commons::makeSharedPcl<pcl::PointXYZ>();
        *broccoli_head = *broccoli_head_cam1 + *broccoli_head_cam2;
        // helpers::converters::pointcloudToStream(*broccoli_head, item_elements[1].pcl_merged);
        return 0;
    }

    int FitBroccoli::fit(std::vector<ItemElement> &item_elements)
    {
        Timer timer("FitBroccoli::fit", _debug);

        if (item_elements.size() != 2)
        {
            _logger->log("FitBroccoli::fit", "method requires exactly 2 pointclouds of elements but received:", item_elements.size(), "dropping further processing");
            for (auto &item_element : item_elements)
                helpers::commons::statisticalOutlierRemovalFilter(item_element.pcl_merged);
            return -1;
        }

        _sortBroccoliParts(item_elements);
        pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_head(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr broccoli_leg(new pcl::PointCloud<pcl::PointXYZ>);
        _filterInputPointclouds(item_elements, broccoli_head, broccoli_leg);

        Eigen::Affine3f leg_pose;
        std::vector<float> leg_dim(2);
        _computeLegPoseAndSize(broccoli_head, broccoli_leg, leg_pose, leg_dim);
        _assignLegData(leg_pose, leg_dim, item_elements[0]);

        Eigen::Affine3f head_pose;
        std::vector<float> head_dim(3);
        _computeHeadPoseAndSize(broccoli_head, head_pose, head_dim);
        _assignHeadData(head_pose, head_dim, item_elements[1]);

        // item_elements[1].primitive_shape["fit_method"] = "octomap";

        return 0;
    }

    int FitBroccoli::_assignLegData(Eigen::Affine3f &leg_pose, std::vector<float> &leg_dim, ItemElement &leg_element)
    {
        json primitive_shape;
        primitive_shape["fit_method"] = "cylinder";
        primitive_shape["dims"]["height"] = leg_dim[0];
        primitive_shape["dims"]["radius"] = leg_dim[1];

        json position;
        Eigen::Translation3f translate(leg_pose.translation());
        position["x"] = translate.x();
        position["y"] = translate.y();
        position["z"] = translate.z();

        json orientation;
        Eigen::Quaternionf quat(leg_pose.rotation());
        orientation["x"] = quat.x();
        orientation["y"] = quat.y();
        orientation["z"] = quat.z();
        orientation["w"] = quat.w();

        leg_element.primitive_shape = primitive_shape;
        leg_element.position = position;
        leg_element.orientation = orientation;
        return 0;
    }

    int FitBroccoli::_assignHeadData(Eigen::Affine3f &head_pose, std::vector<float> &head_dim, ItemElement &head_element)
    {
        json primitive_shape;
        primitive_shape["fit_method"] = "octomap";
        primitive_shape["dims"]["x"] = head_dim[0];
        primitive_shape["dims"]["y"] = head_dim[0];
        primitive_shape["dims"]["z"] = head_dim[0];

        json position;
        Eigen::Translation3f translate(head_pose.translation());
        position["x"] = translate.x();
        position["y"] = translate.y();
        position["z"] = translate.z();

        json orientation;
        Eigen::Quaternionf quat(head_pose.rotation());
        orientation["x"] = quat.x();
        orientation["y"] = quat.y();
        orientation["z"] = quat.z();
        orientation["w"] = quat.w();

        head_element.primitive_shape = primitive_shape;
        head_element.position = position;
        head_element.orientation = orientation;
        return 0;
    }

    int FitBroccoli::_computeHeadPoseAndSize(pcl::PointCloud<pcl::PointXYZ>::Ptr head, Eigen::Affine3f &out_head_pose, std::vector<float> &out_head_dim)
    {
        pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
        feature_extractor.setInputCloud(head);
        feature_extractor.compute();
        pcl::PointXYZ position_OBB;
        Eigen::Matrix3f rotational_matrix_OBB;
        pcl::PointXYZ min_point_OBB;
        pcl::PointXYZ max_point_OBB;
        feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
        out_head_pose = Eigen::Translation3f(position_OBB.getVector3fMap()) * Eigen::Quaternionf(rotational_matrix_OBB);
        out_head_dim = {max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z};
        return 0;
    }

    int FitBroccoli::_computeLegPoseAndSize(pcl::PointCloud<pcl::PointXYZ>::Ptr head, pcl::PointCloud<pcl::PointXYZ>::Ptr leg, Eigen::Affine3f &out_leg_pose, std::vector<float> &out_leg_dim)
    {
        pcl::PointXYZ head_center;
        pcl::computeCentroid(*head, head_center);
        pcl::PointXYZ leg_center;
        pcl::computeCentroid(*leg, leg_center);

        pcl::PointXYZ z_axis;
        z_axis.getVector3fMap() = head_center.getVector3fMap() - leg_center.getVector3fMap();

        Eigen::Quaternionf leg_rot;
        leg_rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), z_axis.getVector3fMap());

        double radius = 0;
        Eigen::Vector4f line_point(leg_center.x, leg_center.y, leg_center.z, 0);
        Eigen::Vector4f line_dir(z_axis.x, z_axis.y, z_axis.z, 0);
        for (auto &pt : leg->points)
        {
            Eigen::Vector4f point(pt.x, pt.y, pt.z, 0);
            radius += sqrt(pcl::sqrPointToLineDistance(point, line_point, line_dir));
        }
        radius /= leg->points.size();
        out_leg_pose = Eigen::Translation3f(leg_center.getVector3fMap()) * leg_rot;

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_leg(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*leg, *transformed_leg, out_leg_pose.inverse());

        float min = transformed_leg->getMatrixXfMap().row(2).minCoeff();
        float max = transformed_leg->getMatrixXfMap().row(2).maxCoeff();

        float cylinder_height = max - min;
        if (out_leg_dim.size() == 0)
            out_leg_dim.resize(2);

        out_leg_dim[0] = cylinder_height;
        out_leg_dim[1] = radius;

        // Update center of leg position
        // Center of a leg is set to be end of the leg opposite to broccoli head
        leg_rot.normalize();
        Eigen::Vector3f leg_center_offset = leg_rot.toRotationMatrix().col(2) * cylinder_height / 2;
        leg_center.getVector3fMap() = leg_center.getVector3fMap() - leg_center_offset;
        out_leg_pose = Eigen::Translation3f(leg_center.getVector3fMap()) * leg_rot;

        return 0;
    }
} // namespace estimate_shape
