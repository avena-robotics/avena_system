#include "helpers_vision/visualization.hpp"

namespace helpers
{
    namespace visualization
    {
        pcl::visualization::PCLVisualizer::Ptr visualize(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_clouds, std::vector<Eigen::Affine3f> affines, pcl::visualization::PCLVisualizer::Ptr viewer, std::string window_name)
        {
            bool spin_once = true;
            if (!viewer)
            {
                viewer = boost::make_shared<pcl::visualization::PCLVisualizer>(window_name);
                spin_once = false;
            }
            for (size_t i = 0; i < point_clouds.size(); ++i)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*point_clouds[i], *color_cloud);

                std::bitset<6> color(i + 1);
                for (auto &pt : color_cloud->points)
                {
                    int divider = point_clouds.size() > 8 ? 2 : 1;
                    pt.r = (color[0] + color[3]) * 255 / divider;
                    pt.g = (color[1] + color[4]) * 255 / divider;
                    pt.b = (color[2] + color[5]) * 255 / divider;
                }
                viewer->addPointCloud(color_cloud, "cloud" + std::to_string(i));
            }
            for (size_t i = 0; i < affines.size(); ++i)
                viewer->addCoordinateSystem(0.1, affines[i], "affine" + std::to_string(i));
            viewer->addCoordinateSystem(0.5);
            if (spin_once)
                viewer->spinOnce();
            else
                viewer->spin();

            for (size_t i = 0; i < point_clouds.size(); ++i)
            {
                viewer->removePointCloud("cloud" + std::to_string(i), 0);
            }
            // viewer.close();
            return viewer;
        }

    } // namespace visualization
} // namespace helpers