#include "fit_octosphere/fit_octosphere.hpp"

namespace estimate_shape
{

    FitOctoSphere::FitOctoSphere(estimate_shape::CameraParameters cam1_params, estimate_shape::CameraParameters cam2_params, std::vector<std::string> &args, json &default_parameters, bool debug)
        : IFitMethod(cam1_params, cam2_params, args, default_parameters, debug)
    {
    }

    FitOctoSphere::~FitOctoSphere()
    {
    }

    int FitOctoSphere::fit(std::vector<ItemElement> &item_elements)
    {
        Timer timer("FitOctoSphere::fit", _debug);

        pcl::PointCloud<pcl::PointXYZ>::Ptr onion_head(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr onion_tail(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Quaternionf head_rot(1, 0, 0, 0);
        json primitive_shape;

        if (_handleUnsupportedElements(item_elements))
            return 1;

        if (item_elements.size() == 0)
            return 1;
        else if (item_elements.size() == 1 && _elements_fit_methods[item_elements[0].element_label] == "octomap")
        {
            json primitive_shape_tail;
            primitive_shape_tail["fit_method"] = "octomap";
            item_elements[0].primitive_shape = primitive_shape_tail;
        }
        else if (item_elements.size() == 1 && _elements_fit_methods[item_elements[0].element_label] == "sphere")
        {
            onion_head = item_elements[0].pcl_merged;
            helpers::commons::statisticalOutlierRemovalFilter(onion_head);
            pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);
            _fitModel(onion_head, pcl::SACMODEL_SPHERE, *sphere_coefficients);
            if (sphere_coefficients->values.size() == 0)
            {
                _logger->log("FitOctoSphere::fit", "sphere_coefficients->values.size() == 0");
                return 1;
            }
            primitive_shape["fit_method"] = "sphere";
            primitive_shape["dims"]["radius"] = sphere_coefficients->values[3];
            Eigen::Vector3f sphere_pose_vec(sphere_coefficients->values[0], sphere_coefficients->values[1], sphere_coefficients->values[2]);
            json pose;
            _assignHeadData(sphere_pose_vec, head_rot, pose);
            item_elements[0].orientation = pose["orientation"];
            item_elements[0].position = pose["position"];
            item_elements[0].primitive_shape = primitive_shape;
        }
        else if (item_elements.size() == 2)
            _fitBothParts(item_elements);

        return 0;
    }

    int FitOctoSphere::setLabelData(std::vector<Label> &labels)
    {
        for (size_t i = 0; i < labels.size(); ++i)
        {
            if (labels[i].fit_method == "octosphere")
            {
                std::vector<std::string> components = labels[i].components;
                for (auto &component : components)
                    _labels.push_back(component);
            }
        }
        for (auto &label_data : labels)
            if (std::find(_labels.begin(), _labels.end(), label_data.label) != _labels.end())
                _elements_fit_methods[label_data.label] = label_data.fit_method;
        return 0;
    }

    int FitOctoSphere::_sortParts(std::vector<ItemElement> &item_elements)
    {
        std::map<std::string, int> order;
        order[_labels[0]] = 0;
        order[_labels[1]] = 1;

        std::sort(item_elements.begin(), item_elements.end(), [&order](ItemElement &a, ItemElement &b) {
            return (order[a.element_label] < order[b.element_label]);
        });

        return 0;
    }

    int FitOctoSphere::_fitBothParts(std::vector<ItemElement> &item_elements)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr onion_head(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr onion_tail(new pcl::PointCloud<pcl::PointXYZ>);
        _sortParts(item_elements);

        onion_head = item_elements[0].pcl_merged;
        onion_tail = item_elements[1].pcl_merged;
        // helpers::converters::streamToPointcloud(item_elements[0].pcl_merged, *onion_head);
        // helpers::converters::streamToPointcloud(item_elements[1].pcl_merged, *onion_tail);
        helpers::commons::statisticalOutlierRemovalFilter(onion_head);
        helpers::commons::statisticalOutlierRemovalFilter(onion_tail);

        pcl::ModelCoefficients::Ptr sphere_coefficients(new pcl::ModelCoefficients);
        _fitModel(onion_head, pcl::SACMODEL_SPHERE, *sphere_coefficients);
        if (sphere_coefficients->values.size() == 0)
        {
            _logger->log("FitOctoSphere::fit", "sphere_coefficients->values.size() = 0");
            return 1;
        }
        std::vector<float> out_obj_dim;
        out_obj_dim.push_back(sphere_coefficients->values[3]);
        Eigen::Vector3f sphere_pose_vec(sphere_coefficients->values[0], sphere_coefficients->values[1], sphere_coefficients->values[2]);

        // Sort by distance to sphere center
        pcl::PointXYZ sphere_pose_pcl(sphere_coefficients->values[0], sphere_coefficients->values[1], sphere_coefficients->values[2]);
        std::sort(onion_tail->points.begin(), onion_tail->points.end(), [&sphere_pose_pcl](pcl::PointXYZ &a, pcl::PointXYZ &b) {
            return pcl::euclideanDistance(a, sphere_pose_pcl) < pcl::euclideanDistance(b, sphere_pose_pcl);
        });
        if (onion_tail->points.size() > 20)
            onion_tail->points.resize(20);

        Eigen::Vector4f tail_centroid;
        pcl::compute3DCentroid(*onion_tail, tail_centroid);
        Eigen::Vector3f onion_axis = tail_centroid.head<3>() - sphere_pose_vec;
        Eigen::Quaternionf head_rot(1, 0, 0, 0);
        head_rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), onion_axis);
        json primitive_shape;

        primitive_shape["fit_method"] = "sphere";
        primitive_shape["dims"]["radius"] = out_obj_dim[0];
        json pose;
        _assignHeadData(sphere_pose_vec, head_rot, pose);
        item_elements[0].orientation = pose["orientation"];
        item_elements[0].position = pose["position"];
        item_elements[0].primitive_shape = primitive_shape;

        json primitive_shape_tail;
        primitive_shape_tail["fit_method"] = "octomap";
        item_elements[1].primitive_shape = primitive_shape_tail;
        return 0;
    }

    int FitOctoSphere::_assignOctomap(ItemElement &item_element)
    {
        json primitive_shape;
        primitive_shape["fit_method"] = "octomap";
        item_element.primitive_shape = primitive_shape;
        return 0;
    }

    int FitOctoSphere::_handleUnsupportedElements(std::vector<ItemElement> &item_elements)
    {
        int unsupported_elements = 0;
        for (size_t i = 0; i < item_elements.size(); i++)
            if (std::find(_labels.begin(), _labels.end(), item_elements[i].element_label) == _labels.end())
            {
                _assignOctomap(item_elements[i]);
                unsupported_elements = 1;
            }
        return unsupported_elements;
    }

    int FitOctoSphere::_assignHeadData(const Eigen::Vector3f &pos, const Eigen::Quaternionf &rot, json &pose)
    {
        pose["position"]["x"] = pos.x();
        pose["position"]["y"] = pos.y();
        pose["position"]["z"] = pos.z();
        pose["orientation"]["x"] = rot.x();
        pose["orientation"]["y"] = rot.y();
        pose["orientation"]["z"] = rot.z();
        pose["orientation"]["w"] = rot.w();
        return 0;
    }

    int FitOctoSphere::_fitModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, uint8_t model_type, pcl::ModelCoefficients &out_model_coefficients)
    {
        if (cloud->points.size() == 0)
        {
            _logger->log("FitOctoSphere::_fitModel", "no points in input pointcloud");
            return 1;
        }
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setNumberOfThreads(std::thread::hardware_concurrency() / 2);
        pcl::PointIndices model_inlierIndices;
        segmentation.setInputCloud(cloud);
        segmentation.setModelType(model_type);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setDistanceThreshold(0.005);
        segmentation.setOptimizeCoefficients(true);
        segmentation.setRadiusLimits(0.01, 0.1);
        segmentation.setEpsAngle(90 / (180 / 3.141592654));
        segmentation.setMaxIterations(100000);
        segmentation.segment(model_inlierIndices, out_model_coefficients);
        return 0;
    }

} // namespace estimate_shape
