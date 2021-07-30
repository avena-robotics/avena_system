#include "estimate_shape_fit_box/fit_box.hpp"

namespace estimate_shape
{

    FitBox::FitBox(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters)
    {
    }

    FitBox::~FitBox()
    {
    }

    int FitBox::_getBoxData(std::vector<ItemElement> &item_elements, pcl::PointCloud<pcl::PointXYZ>::Ptr box_ptcld)
    {
        for (auto &item_element : item_elements)
            *box_ptcld += *item_element.pcl_merged;
        // helpers::vision::passThroughFilter(box_ptcld, "z", 0.001, 2);
        // helpers::vision::statisticalOutlierRemovalFilter(box_ptcld,10,1.80);
        return 0;
    }

    int FitBox::fit(Item &item)
    {
        Timer timer("FitBox::fit", LOGGER);

        pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        _getBoxData(item.item_elements, box_cloud);

        pcl::ModelCoefficients model_coefficients;

        pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_planes(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointIndices plane_indices;
        for (int i = 0; i < 6; i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            _fitPlane(box_cloud, plane_indices, model_coefficients);
            if (plane_indices.indices.size() < 20)
                break;
            helpers::vision::extract(plane_indices.indices, false, box_cloud, plane_cloud);
            helpers::vision::extract(plane_indices.indices, true, box_cloud);
            *extracted_planes += *plane_cloud;
        }

        box_cloud = extracted_planes;

        Eigen::Affine3f box_affine;
        if (_findInitialAffine(box_cloud, box_affine))
        {
            LOG_DEBUG("FitBox::fit: Error while finding initial affine pose. Exiting further processing");
            return -1;
        }

        std::vector<Dimensions> dim(3);
        _alignBoxAxes(box_cloud, box_affine, dim);
        _assignBoxData(dim, box_affine, item);

        return 0;
    }

    int FitBox::_findInitialAffine(pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud, Eigen::Affine3f &out_initial_affine)
    {
        // pcl::PointIndicesPtr indicies(new pcl::PointIndices);
        pcl::ModelCoefficients coeffs;
        _fitPlane(box_cloud, coeffs);
        if (coeffs.values.size() == 0)
        {
            LOG_DEBUG("FitBox::_findInitialAffine: No model coefficients for fitting plane.");
            return -1;
        }
        Eigen::Vector3f normal(coeffs.values.at(0), coeffs.values.at(1), coeffs.values.at(2));
        Eigen::Quaternionf rot;
        rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), normal);
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *plane_cloud = *box_cloud;
        pcl::PointXYZ center;
        pcl::computeCentroid(*box_cloud, center);
        Eigen::Translation3f box_center_trans(center.getVector3fMap());

        Eigen::Affine3f trans = box_center_trans * rot;
        pcl::transformPointCloud(*plane_cloud, *plane_cloud, trans.inverse());

        // TODO: Check this mystery
        pcl::Indices indices;
        pcl::removeNaNFromPointCloud(*plane_cloud, *plane_cloud, indices);

        plane_cloud->getMatrixXfMap().row(2) = Eigen::VectorXf::Zero(plane_cloud->points.size());
        pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
        helpers::vision::hull2D(plane_cloud, hull);

        double min_area = std::numeric_limits<double>::max();
        Eigen::Quaternionf q;
        for (size_t i = 0; i < hull->points.size() - 1; ++i)
        {
            Eigen::Matrix3f rotation = helpers::vision::findRotationFromHullPoints(hull->points[i], hull->points[i + 1]);
            Eigen::Matrix3f plane_rotation = Eigen::Matrix3f::Identity();
            pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            helpers::vision::rotateHull(rotation, hull, projected_cloud);
            // Compute min/max
            double x_min = projected_cloud->getMatrixXfMap().row(0).minCoeff();
            double x_max = projected_cloud->getMatrixXfMap().row(0).maxCoeff();
            double y_min = projected_cloud->getMatrixXfMap().row(1).minCoeff();
            double y_max = projected_cloud->getMatrixXfMap().row(1).maxCoeff();
            // Is this the best estimate?
            double area = (x_max - x_min) * (y_max - y_min);
            if (area < min_area)
            {
                min_area = area;
                q = Eigen::Quaternionf((plane_rotation.inverse() * rotation.inverse()));
            }
        }
        rot = rot * q;
        out_initial_affine = box_center_trans * rot;
        return 0;
    }

    int FitBox::_fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices &out_model_inlierIndices, pcl::ModelCoefficients &out_model_coefficients)
    {
        if (cloud->points.size() == 0)
        {
            LOG_DEBUG("FitBox::_fitPlane: No points to fit plane");
            return 1;
        }
        // estimate points that belong to plane
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setInputCloud(cloud);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setDistanceThreshold(0.002);
        segmentation.setOptimizeCoefficients(false);
        segmentation.segment(out_model_inlierIndices, out_model_coefficients);
        if (out_model_coefficients.values.size() == 0)
            return 1;
        return 0;
    }

    int FitBox::_fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::ModelCoefficients &out_model_coefficients)
    {
        pcl::PointIndices model_inlierIndices;
        return _fitPlane(cloud, model_inlierIndices, out_model_coefficients);
    }

    int FitBox::_alignBoxAxes(pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud, Eigen::Affine3f &box_affine, std::vector<Dimensions> &out_dim)
    {
        pcl::transformPointCloud(*box_cloud, *box_cloud, box_affine.inverse());

        float min_x = box_cloud->getMatrixXfMap().row(0).minCoeff();
        float max_x = box_cloud->getMatrixXfMap().row(0).maxCoeff();
        float min_y = box_cloud->getMatrixXfMap().row(1).minCoeff();
        float max_y = box_cloud->getMatrixXfMap().row(1).maxCoeff();
        float min_z = box_cloud->getMatrixXfMap().row(2).minCoeff();
        float max_z = box_cloud->getMatrixXfMap().row(2).maxCoeff();

        out_dim.clear();
        out_dim.resize(3);
        out_dim[0] = Dimensions(min_x, max_x);
        out_dim[1] = Dimensions(min_y, max_y);
        out_dim[2] = Dimensions(min_z, max_z);

        Eigen::Quaternionf rot(box_affine.rotation());

        auto [min_it, max_it] = std::minmax_element(out_dim.begin(), out_dim.end());
        int min_idx = min_it - out_dim.begin();
        int max_idx = max_it - out_dim.begin();
        if (max_idx == 0)
        {
            rot = rot * Eigen::Quaternionf(helpers::vision::assignRotationMatrixAroundY(M_PI_2));
            if (min_idx == 1)
                rot = rot * Eigen::Quaternionf(helpers::vision::assignRotationMatrixAroundZ(M_PI_2));
        }
        else if (max_idx == 1)
        {
            rot = rot * Eigen::Quaternionf(helpers::vision::assignRotationMatrixAroundX(M_PI_2));
            if (min_idx == 2)
                rot = rot * Eigen::Quaternionf(helpers::vision::assignRotationMatrixAroundZ(M_PI_2));
        }
        else
        {
            if (min_idx == 1)
                rot = rot * Eigen::Quaternionf(helpers::vision::assignRotationMatrixAroundZ(M_PI_2));
        }
        // Reinforce box center
        Eigen::Vector3f temp_displacement(out_dim[0].center_displacement, out_dim[1].center_displacement, out_dim[2].center_displacement);
        temp_displacement = box_affine * temp_displacement;

        box_affine = Eigen::Translation3f(temp_displacement) * rot;
        return 0;
    }

    int FitBox::_assignBoxData(std::vector<Dimensions> &dim, const Eigen::Affine3f &box_affine, Item &item)
    {
        std::sort(dim.begin(), dim.end());

        item.pose = box_affine;

        json part_description;
        part_description["fit_method"] = "box";
        part_description["spawn_method"] = "box";
        part_description["part_name"] = item.label;
        part_description["dims"]["x"] = dim[0].size;
        part_description["dims"]["y"] = dim[1].size;
        part_description["dims"]["z"] = dim[2].size;

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

} // namespace estimate_shape
