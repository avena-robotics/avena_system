#include "estimate_shape_fit_cutting_board/fit_cutting_board.hpp"

namespace estimate_shape
{
    FitCuttingBoard::FitCuttingBoard(const std::vector<CameraParameters> &camera_params, const std::vector<std::string> &args, const json &default_parameters)
        : IFitMethod(camera_params, args, default_parameters),
          _show(false)
    {
    }

    FitCuttingBoard::~FitCuttingBoard()
    {
    }
    int FitCuttingBoard::_removePointsAboveBoard(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld, bool /*sac_show*/)
    {
        // Timer timer("FitCuttingBoard::_reinforceBoardOrientation", LOGGER);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        _calculatePlanePosition(ptcld, coefficients);
        Eigen::Vector3f normal(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        Eigen::Quaternionf z_rot;
        z_rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), normal);
        pcl::PointXYZ center;
        pcl::computeCentroid(*ptcld, center);
        Eigen::Affine3f initial_plane_aff = Eigen::Translation3f(center.getVector3fMap()) * z_rot;
        pcl::transformPointCloud(*ptcld, *ptcld, initial_plane_aff.inverse());
        helpers::vision::passThroughFilter(ptcld, "z", -0.1, coefficients->values[3] + 0.02);
        if (ptcld->points.size() == 0)
            return 1;
        pcl::transformPointCloud(*ptcld, *ptcld, initial_plane_aff);
        return 0;
    }

    int FitCuttingBoard::_fitBox(Item &item)
    {
        FitBox fit_box(_camera_params, _args, _default_parameters);
        fit_box.fit(item);
        return 0;
    }

    // Eigen::Affine3f FitCuttingBoard::_readAffineFromItem(const Item &item)
    // {
    //     Eigen::Vector3f vec = helpers::vision::assignPositionFromJson(item.position);
    //     Eigen::Quaternionf rot = helpers::vision::assignQuaternionFromJson(item.orientation);
    //     Eigen::Affine3f item_aff = Eigen::Translation3f(vec) * rot;
    //     return item_aff;
    // }

    int FitCuttingBoard::_reinforceBoardOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr item_cloud, Item &item, float cut_distance)
    {
        // Timer timer("FitCuttingBoard::_reinforceBoardOrientation", LOGGER);
        Eigen::Affine3f item_aff = item.pose;
        pcl::PointCloud<pcl::PointXYZ>::Ptr top_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*item_cloud, *top_cloud, item_aff.inverse()); //tutaj
        float max_z = top_cloud->getMatrixXfMap().row(2).maxCoeff();
        helpers::vision::passThroughFilter(top_cloud, "z", max_z - cut_distance, max_z);
        pcl::PointXYZ top_center;
        pcl::computeCentroid(*top_cloud, top_center);

        Eigen::Quaternionf rot_adjust;
        rot_adjust.setFromTwoVectors(Eigen::Vector3f::UnitZ(), top_center.getVector3fMap());
        Eigen::Quaternionf rot(item_aff.rotation());
        rot = rot * rot_adjust;
        item.pose = Eigen::Translation3f(item.pose.translation()) * rot;
        return 0;
    }

    int FitCuttingBoard::_fixCuttingBoardOrientation(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld, Item &item)
    {
        // Timer timer("FitCuttingBoard::_fixCuttingBoardOrientation", LOGGER);
        Eigen::Affine3f item_aff = item.pose;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_item_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*ptcld, *transformed_item_cloud, item_aff.inverse());
        float min_z = transformed_item_cloud->getMatrixXfMap().row(2).minCoeff();
        float max_z = transformed_item_cloud->getMatrixXfMap().row(2).maxCoeff();

        pcl::PointCloud<pcl::PointXYZ>::Ptr top_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr bottom_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        float cut_distance = 0.01;
        float top_y_size = 0;
        float bottom_y_size = 0;
        float z_size = transformed_item_cloud->getMatrixXfMap().row(2).maxCoeff() - transformed_item_cloud->getMatrixXfMap().row(2).minCoeff();
        while (cut_distance < z_size / 2)
        {
            *top_cloud = *transformed_item_cloud;
            *bottom_cloud = *transformed_item_cloud;
            helpers::vision::passThroughFilter(top_cloud, "z", max_z - cut_distance, 2);
            helpers::vision::passThroughFilter(bottom_cloud, "z", -2, min_z + cut_distance);

            top_y_size = top_cloud->getMatrixXfMap().row(1).maxCoeff() - top_cloud->getMatrixXfMap().row(1).minCoeff();
            bottom_y_size = bottom_cloud->getMatrixXfMap().row(1).maxCoeff() - bottom_cloud->getMatrixXfMap().row(1).minCoeff();
            if (top_y_size < bottom_y_size * 0.4)
                break;
            if (bottom_y_size < top_y_size * 0.4)
                break;
            cut_distance += 0.01;
        }
        Eigen::Quaternionf box_rot(item_aff.rotation());
        if (top_y_size > bottom_y_size)
            box_rot = box_rot * helpers::vision::assignRotationMatrixAroundX(M_PI);
        item.pose = Eigen::Translation3f(item.pose.translation()) * box_rot;
        if (top_y_size < bottom_y_size * 0.4 || bottom_y_size < top_y_size * 0.4)
            _reinforceBoardOrientation(ptcld, item, cut_distance);
        return 0;
    }

    int FitCuttingBoard::fit(Item &item)
    {
        Timer timer("FitCuttingBoard::fit", LOGGER);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cutting_board_cloud = item.item_elements[0].pcl_merged;
        // helpers::converters::streamToPointcloud(item_elements[0].pcl_merged, *cutting_board_cloud);

        LOG_DEBUG_STREAM("Loading cutting board pointcloud from database with item_id " << item.item_elements[0].item_id);
        if (_removePointsAboveBoard(cutting_board_cloud, _show))
            return 1;
        _removeOutliers(cutting_board_cloud);

        // helpers::converters::pointcloudToStream(*cutting_board_cloud, item_elements[0].pcl_merged);
        _fitBox(item);
        _fixCuttingBoardOrientation(cutting_board_cloud, item);

        Eigen::Affine3f item_aff = item.pose;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cutting_board_cloud, *transformed_cloud, item_aff.inverse());

        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_map = _cuttingBoardSplit(transformed_cloud);
        _assignData(cloud_map, item);

        return 0;
    }

    int FitCuttingBoard::_assignData(std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_map, Item &item)
    {
        // Timer timer("FitCuttingBoard::_assignData", LOGGER);
        pcl::PointXYZ center;
        std::vector<float> dims(3);
        Eigen::Affine3f item_aff = item.pose;
        Eigen::Quaternionf quat(item_aff.rotation());

        std::vector<json> parts_description;
        for (auto &part : _parts)
        {
            json part_description;
            dims[0] = cloud_map[part]->getMatrixXfMap().row(0).maxCoeff() - cloud_map[part]->getMatrixXfMap().row(0).minCoeff();
            dims[1] = cloud_map[part]->getMatrixXfMap().row(1).maxCoeff() - cloud_map[part]->getMatrixXfMap().row(1).minCoeff();
            dims[2] = cloud_map[part]->getMatrixXfMap().row(2).maxCoeff() - cloud_map[part]->getMatrixXfMap().row(2).minCoeff();
            part_description["dims"]["x"] = dims[0];
            part_description["dims"]["y"] = dims[1];
            part_description["dims"]["z"] = dims[2];
            part_description["spawn_method"] = "box";
            part_description["fit_method"] = "cuttingboard";
            part_description["part_name"] = part;
            pcl::transformPointCloud(*cloud_map[part], *cloud_map[part], item_aff);
            pcl::computeCentroid(*cloud_map[part], center);
            Eigen::Vector3f position = center.getVector3fMap();
            part_description["pose"]["orientation"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"w", 1.0}};
            if (part == "board") // center of board part is a center of a whole cutting board
            {
                item.pose = Eigen::Translation3f(position) * quat;
                part_description["pose"]["position"] = {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}};
            }
            else
                part_description["pose"]["position"] = helpers::vision::assignJsonFromPosition(position);
            parts_description.push_back(part_description);
        }

        // Update position of handle to be relative to center of a cutting board (which is center of board part of cutting board)
        json &handle_part_description = parts_description[0];
        Eigen::Vector3f handle_position = helpers::vision::assignPositionFromJson(handle_part_description["pose"]["position"]);
        pcl::transformPoint(handle_position, handle_position, item.pose.inverse());
        handle_part_description["pose"]["position"] = helpers::vision::assignJsonFromPosition(handle_position);

        item.item_elements[0].parts_description = parts_description;
        return 0;
    }

    bool FitCuttingBoard::_removeOutliers(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld)
    {
        // Timer timer("FitCuttingBoard::_removeOutliers", LOGGER);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(ptcld);
        sor.setMeanK(30);            // 20
        sor.setStddevMulThresh(2.5); //2.5
        sor.filter(*ptcld);
        return true;
    }

    bool FitCuttingBoard::_calculatePlanePosition(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld, pcl::ModelCoefficients::Ptr coefficients)
    {
        // Timer timer("FitCuttingBoard::_calculatePlanePosition", LOGGER);
        pcl::PointIndices inliers;
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(ptcld);
        seg.segment(inliers, *coefficients);
        return true;
    }
    std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> FitCuttingBoard::_cuttingBoardSplit(pcl::PointCloud<pcl::PointXYZ>::Ptr ptcld)
    {
        // Timer timer("FitCuttingBoard::_cuttingBoardSplit", LOGGER);
        pcl::PointCloud<pcl::PointXYZ>::Ptr handle_ptcld(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr board_ptcld(new pcl::PointCloud<pcl::PointXYZ>);
        float cut_distance = 0.01;
        float min_z = ptcld->getMatrixXfMap().row(2).minCoeff();
        float max_z = ptcld->getMatrixXfMap().row(2).maxCoeff();
        float z_size = ptcld->getMatrixXfMap().row(2).maxCoeff() - ptcld->getMatrixXfMap().row(2).minCoeff();
        float board_width = ptcld->getMatrixXfMap().row(1).maxCoeff() - ptcld->getMatrixXfMap().row(1).minCoeff();
        while (cut_distance < z_size / 2)
        {
            *handle_ptcld = *ptcld;
            *board_ptcld = *ptcld;

            helpers::vision::passThroughFilter(handle_ptcld, "z", max_z - cut_distance, max_z);
            helpers::vision::passThroughFilter(board_ptcld, "z", min_z, max_z - cut_distance);
            float handle_width = handle_ptcld->getMatrixXfMap().row(1).maxCoeff() - handle_ptcld->getMatrixXfMap().row(1).minCoeff();

            if (handle_width < board_width / 2 && handle_width > board_width / 3)
                break;

            cut_distance += 0.01;
        }
        std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_map;
        cloud_map[_parts[0]] = handle_ptcld;
        cloud_map[_parts[1]] = board_ptcld;

        return cloud_map;
    }
} // namespace estimate_shape
