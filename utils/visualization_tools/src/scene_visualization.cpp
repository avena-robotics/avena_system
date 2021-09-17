#include "visualization_tools/scene_visualization.hpp"

namespace visualization_tools
{
    SceneVisualization::SceneVisualization(const rclcpp::NodeOptions &options)
        : Node("scene_visualization", options)
    {
        helpers::commons::setLoggerLevel(get_logger(), "debug");
        _getParametersFromServer();
        _table_marker_pub = create_publisher<visualization_msgs::msg::Marker>("scene_markers", 1);

        auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        _octomap_changed_sub = create_subscription<OctomapChangeFlag>("scene_change_flag", qos_settings, std::bind(&SceneVisualization::_octomapUpdated, this, std::placeholders::_1));
        _octomap_select_client = create_client<OctomapSelect>("scene_select");
    }

    int SceneVisualization::_getParametersFromServer()
    {
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");
        while (true)
        {
            auto parameters = helpers::commons::getParameters({"areas"});
            if (parameters.empty())
                continue;

            _areas_parameters = parameters["areas"];
            break;
        }

        RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        return 0;
    }

    void SceneVisualization::_octomapUpdated(const OctomapChangeFlag::SharedPtr /*octomap_change_flag_msg*/)
    {
        RCLCPP_INFO(get_logger(), "Octomap updated on the server");
        std::thread([this]()
                    {
                        // Read octomap of the scene
                        auto octomap_req = std::make_shared<OctomapSelect::Request>();
                        octomap_req->time_stamp.data = 0.0; // Currently it does not matter
                        auto octomap_res = _octomap_select_client->async_send_request(octomap_req);
                        if (octomap_res.wait_for(std::chrono::seconds(1)) != std::future_status::ready)
                        {
                            RCLCPP_WARN(get_logger(), "Cannot read octomap from server");
                            return;
                        }

                        auto ros_octomap = octomap_res.get()->data.octomap.scene_octomap;
                        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = helpers::vision::makeSharedPcl<pcl::PointXYZ>();
                        helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(ros_octomap, point_cloud);

                        // Create mesh to visualize exacly the same as in physics server
                        const float grid_size = 0.025;
                        std::vector<pcl::Vertices> triangles;
                        if (_createMeshFromPointCloud(point_cloud, grid_size, triangles))
                        {
                            RCLCPP_DEBUG(get_logger(), "There is no points in input cloud");
                        }

                        visualization_msgs::msg::Marker octomap_points_marker;
                        octomap_points_marker.header.frame_id = "world";
                        octomap_points_marker.ns = "octomap";
                        octomap_points_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
                        octomap_points_marker.action = visualization_msgs::msg::Marker::ADD;
                        octomap_points_marker.lifetime = rclcpp::Duration::from_seconds(0);
                        octomap_points_marker.pose.orientation.w = 1;
                        octomap_points_marker.scale.x = 1.0;
                        octomap_points_marker.scale.y = 1.0;
                        octomap_points_marker.scale.z = 1.0;
                        octomap_points_marker.color.r = 1.0;
                        octomap_points_marker.color.g = 1.0;
                        octomap_points_marker.color.b = 1.0;
                        octomap_points_marker.color.a = 1.0;
                        for (auto &vertex : triangles)
                        {
                            for (auto &pt_id : vertex.vertices)
                            {
                                geometry_msgs::msg::Point mesh_triangle_pt;
                                mesh_triangle_pt.x = point_cloud->points[pt_id].x;
                                mesh_triangle_pt.y = point_cloud->points[pt_id].y;
                                mesh_triangle_pt.z = point_cloud->points[pt_id].z;
                                octomap_points_marker.points.push_back(mesh_triangle_pt);
                            }
                        }
                        _table_marker_pub->publish(octomap_points_marker);
                    })
            .detach();
    }

    int SceneVisualization::_createMeshFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &point_cloud, const float &grid_size, std::vector<pcl::Vertices> &out_triangles)
    {
        if (point_cloud->size() == 0)
            return 1;

        // RCLCPP_WARN_STREAM(get_logger(), "Amount of points before voxelization: " << point_cloud->size());
        {
            helpers::Timer timer("Input point cloud voxelization", get_logger());
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(point_cloud);
            voxel_filter.setLeafSize(grid_size, grid_size, grid_size);
            voxel_filter.filter(*point_cloud);
        }
        // RCLCPP_WARN_STREAM(get_logger(), "Amount of points after voxelization: " << point_cloud->size());

        {
            helpers::Timer timer("Create mesh with PCL", get_logger());
            // Normal estimation*
            pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(point_cloud);
            n.setInputCloud(point_cloud);
            n.setSearchMethod(tree);
            n.setKSearch(20);
            n.setNumberOfThreads(std::thread::hardware_concurrency());
            n.compute(*normals);
            //* normals should not contain the point normals + surface curvatures

            // Concatenate the XYZ and normal fields*
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);
            //* cloud_with_normals = cloud + normals

            // Create search tree*
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud(cloud_with_normals);

            // Initialize objects
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius(grid_size * 2);

            // Set typical values for the parameters
            gp3.setMu(2.5);
            gp3.setMaximumNearestNeighbors(100);
            gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
            gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
            gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
            gp3.setNormalConsistency(false);

            // Get result
            gp3.setInputCloud(cloud_with_normals);
            gp3.setSearchMethod(tree2);
            gp3.reconstruct(out_triangles);
        }

        // {
        //     auto v0 = helpers::visualization::visualize({point_cloud}, {}, nullptr, "cloud");
        //     // Visualize
        //     pcl::PolygonMesh mesh;
        //     mesh.polygons = triangles;
        //     pcl::toPCLPointCloud2(*point_cloud, mesh.cloud);
        //     pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
        //     viewer->setBackgroundColor(0, 0, 0);
        //     viewer->addPolygonMesh(mesh, "meshes", 0);
        //     viewer->addCoordinateSystem(1.0);
        //     viewer->initCameraParameters();
        //     viewer->spin();
        // }
        // auto viewer = helpers::visualization::visualize({pcl_octomap});

        return 0;
    }

} // namespace visualization_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(visualization_tools::SceneVisualization)
