// ___CPP___
#include <filesystem>
#include <nlohmann/json.hpp>
#include <fstream>

// ___Other___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/gp3.h>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// ___Avena___
#include <helpers_vision/helpers_vision.hpp>
#include <custom_interfaces/srv/data_store_items_insert.hpp>
#include <custom_interfaces/srv/data_store_scene_insert.hpp>

using json = nlohmann::json;

// Util command line argument parser
class CmdOptionsParser
{
public:
    explicit CmdOptionsParser(int argc, char **argv)
    {
        for (int i = 0; i < argc; i++)
            cmd_args.push_back(argv[i]);
    }

    bool optionExists(const std::string &option)
    {
        return std::find(cmd_args.begin(), cmd_args.end(), option) != cmd_args.end();
    }

private:
    std::vector<std::string> cmd_args;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("load_scene_to_datastore");
    CmdOptionsParser cmd_parser(argc, argv);

    // Load JSON with simulation objects
    const auto sim_items_file_pkg = std::filesystem::path(ament_index_cpp::get_package_share_directory("spawn_collision_items"));
    const auto sim_items_file_path = sim_items_file_pkg / "data" / "simulation_scene.json";
    std::ifstream sim_items_file(sim_items_file_path.string());

    json items = json::parse(sim_items_file);
    custom_interfaces::msg::Items ros_items;
    try
    {
        for (auto &item : items["items"])
        {
            custom_interfaces::msg::Item ros_item;
            ros_item.id = item["id"].get<int>();
            ros_item.label = item["label"].get<std::string>();
            ros_item.pose.position.x = item["pose"]["position"]["x"].get<float>();
            ros_item.pose.position.y = item["pose"]["position"]["y"].get<float>();
            ros_item.pose.position.z = item["pose"]["position"]["z"].get<float>();
            ros_item.pose.orientation.x = item["pose"]["orientation"]["x"].get<float>();
            ros_item.pose.orientation.y = item["pose"]["orientation"]["y"].get<float>();
            ros_item.pose.orientation.z = item["pose"]["orientation"]["z"].get<float>();
            ros_item.pose.orientation.w = item["pose"]["orientation"]["w"].get<float>();

            for (auto item_element : item["item_elements"])
            {
                custom_interfaces::msg::ItemElement ros_item_element;
                ros_item_element.id = item_element["id"].get<int>();
                ros_item_element.label = item_element["label"].get<std::string>();

                for (auto part_description : item_element["parts_description"])
                {
                    ros_item_element.parts_description.push_back(part_description.dump());
                }

                ros_item.item_elements.push_back(ros_item_element);
            }

            ros_items.items.push_back(ros_item);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Error occured while parsing simulation scene JSON. Error: " << e.what());
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Simulation scene JSON parsed successfully");
    ros_items.header.stamp = node->now();

    {
        // Write items to data store
        RCLCPP_INFO(node->get_logger(), "Writing items to server");
        auto items_insert_client = node->create_client<custom_interfaces::srv::DataStoreItemsInsert>("items_insert");
        RCLCPP_INFO(node->get_logger(), "Waiting for data store server for 3 seconds...");
        if (!items_insert_client->wait_for_service(std::chrono::seconds(3)))
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Data store server is not available. Exiting...");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "Data store server is available");

        auto request = std::make_shared<custom_interfaces::srv::DataStoreItemsInsert::Request>();
        request->data = ros_items;

        auto data_store_future = items_insert_client->async_send_request(request);
        RCLCPP_INFO(node->get_logger(), "Trying to write data to store for 5 seconds...");
        auto data_store_future_result = rclcpp::spin_until_future_complete(node, data_store_future, std::chrono::seconds(5));
        if (rclcpp::ok() && data_store_future_result == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Successfully write items to data store");
        }
        else
        {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Error occured during writing items to data store. Exiting...");
            return 1;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud = helpers::vision::makeSharedPcl<pcl::PointXYZ>();
    if (cmd_parser.optionExists("clear_octomap"))
    {
        RCLCPP_INFO(node->get_logger(), "Clearing octomap");
    }
    else
    {
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_real_distribution<float> distr(0, 0.1);

        // Write scene octomap to data store
        RCLCPP_INFO(node->get_logger(), "Writing scene octomap to server");
        {
            // First box
            const float x_min = 0.4;
            const float x_max = x_min + 0.2;
            const float y_min = 0.4;
            const float y_max = y_min + 0.3;
            const float z_min = 0.0;
            const float z_max = z_min + 0.3;
            const float inc = 0.005;

            for (float x = x_min; x <= x_max; x += inc)
            {
                for (float y = y_min; y <= y_max; y += inc)
                {
                    for (float z = z_min; z <= z_max; z += inc)
                    {
                        if (std::fabs(x - x_min) < 0.002 || std::fabs(x - x_max) < 0.002 ||
                            std::fabs(y - y_min) < 0.002 || std::fabs(y - y_max) < 0.002 ||
                            std::fabs(z - z_min) < 0.002 || std::fabs(z - z_max) < 0.002)
                        {
                            pcl_cloud->push_back(pcl::PointXYZ(x + distr(eng), y + distr(eng), z + distr(eng)));
                        }
                    }
                }
            }
        }

        {
            // Second box
            const float x_min = 0.2;
            const float x_max = x_min + 0.2;
            const float y_min = -0.5;
            const float y_max = y_min + 0.3;
            const float z_min = 0.0;
            const float z_max = z_min + 0.2;
            const float inc = 0.005;

            for (float x = x_min; x <= x_max; x += inc)
            {
                for (float y = y_min; y <= y_max; y += inc)
                {
                    for (float z = z_min; z <= z_max; z += inc)
                    {
                        if (std::fabs(x - x_min) < 0.002 || std::fabs(x - x_max) < 0.002 ||
                            std::fabs(y - y_min) < 0.002 || std::fabs(y - y_max) < 0.002 ||
                            std::fabs(z - z_min) < 0.002 || std::fabs(z - z_max) < 0.002)
                        {
                            pcl_cloud->push_back(pcl::PointXYZ(x + distr(eng), y + distr(eng), z + distr(eng)));
                        }
                    }
                }
            }
        }

    //     {
    //         // Load point cloud from file
    //         pcl::io::loadPLYFile("/home/avena/Downloads/ptcld_combined_14.ply", *pcl_cloud);
    //         pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_temp = helpers::vision::makeSharedPcl<pcl::PointXYZ>();
    //         pcl::io::loadPLYFile("/home/avena/Downloads/ptcld_combined_24.ply", *pcl_cloud_temp);
    //         *pcl_cloud += *pcl_cloud_temp;
    //     }

    //     // auto viewer = helpers::visualization::visualize({pcl_cloud});

    //     const float voxel_grid_size = 0.025;
    //     pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    //     voxel_filter.setInputCloud(pcl_cloud);
    //     voxel_filter.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    //     voxel_filter.filter(*pcl_cloud);

    //     // Normal estimation*
    //     pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    //     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //     tree->setInputCloud(pcl_cloud);
    //     n.setInputCloud(pcl_cloud);
    //     n.setSearchMethod(tree);
    //     n.setKSearch(20);
    //     n.setNumberOfThreads(std::thread::hardware_concurrency());
    //     n.compute(*normals);
    //     //* normals should not contain the point normals + surface curvatures

    //     // Concatenate the XYZ and normal fields*
    //     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    //     pcl::concatenateFields(*pcl_cloud, *normals, *cloud_with_normals);
    //     //* cloud_with_normals = cloud + normals

    //     // Create search tree*
    //     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    //     tree2->setInputCloud(cloud_with_normals);

    //     // Initialize objects
    //     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    //     std::vector<pcl::Vertices> triangles;

    //     // Set the maximum distance between connected points (maximum edge length)
    //     gp3.setSearchRadius(voxel_grid_size + 0.01);

    //     // Set typical values for the parameters
    //     gp3.setMu(2.5);
    //     gp3.setMaximumNearestNeighbors(100);
    //     gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    //     gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    //     gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    //     gp3.setNormalConsistency(false);

    //     // Get result
    //     gp3.setInputCloud(cloud_with_normals);
    //     gp3.setSearchMethod(tree2);
    //     gp3.reconstruct(triangles);

    //     if (cmd_parser.optionExists("visualize"))
    //     {
    //         // Visualize
    //         pcl::PolygonMesh mesh;
    //         mesh.polygons = triangles;
    //         pcl::toPCLPointCloud2(*pcl_cloud, mesh.cloud);
    //         pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer"));
    //         viewer->setBackgroundColor(0, 0, 0);
    //         viewer->addPolygonMesh(mesh, "meshes", 0);
    //         viewer->addCoordinateSystem(1.0);
    //         // viewer->initCameraParameters();
    //         viewer->spin();
    //     }
    }

    auto scene_octomap_client = node->create_client<custom_interfaces::srv::DataStoreSceneInsert>("scene_insert");
    RCLCPP_INFO(node->get_logger(), "Waiting for data store server for 3 seconds...");
    if (!scene_octomap_client->wait_for_service(std::chrono::seconds(3)))
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Data store server is not available. Exiting...");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Data store server is available");

    auto request = std::make_shared<custom_interfaces::srv::DataStoreSceneInsert::Request>();

    custom_interfaces::msg::SceneData scene_data;
    helpers::converters::pclToRosPtcld<pcl::PointXYZ>(pcl_cloud, scene_data.octomap.scene_octomap);
    request->data = scene_data;

    auto data_store_future = scene_octomap_client->async_send_request(request);
    RCLCPP_INFO(node->get_logger(), "Trying to write data to store for 5 seconds...");
    auto data_store_future_result = rclcpp::spin_until_future_complete(node, data_store_future, std::chrono::seconds(5));
    if (rclcpp::ok() && data_store_future_result == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Successfully write scene octomap to data store");
    }
    else
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), "Error occured during writing scene octomap to data store. Exiting...");
        return 1;
    }

    return 0;
}
