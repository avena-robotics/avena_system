#ifndef CCV_STRUCTURES_H
#define CCV_STRUCTURES_H

#include <iostream>
#include <vector>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
// #include <mysql_connector.h>
// #include <mysql_connector_structs.h>

#include "utilities.h"
#include "pcl_methods/create_ptcld.hpp"
// #include "helpers.hpp"

// using json = nlohmann::json;

struct Debug
{
    TimeVar start;
    bool debug;
    std::string time_print_msg;
    Debug(std::string msg_to_print, bool debug, std::string time_print_msg = "Execution time:")
        : debug(debug),
          time_print_msg(time_print_msg)
    {
        if (debug)
            std::cout << msg_to_print << std::endl;
        start = timeNow();
    }

    ~Debug()
    {
        auto stop = std::chrono::system_clock::now();
        (void)stop;
        double duration = duration(timeNow() - start);
        if (debug)
            printf("%s %.0f[ms]\n", time_print_msg.c_str(), duration);
    }
};

struct item_cam_t
{
    item_cam_t()
        : pcl_data(new pcl::PointCloud<pcl::PointXYZRGB>),
          pcl_shadow(new pcl::PointCloud<pcl::PointXYZRGB>){};
    uint32_t item_cam_id;
    uint32_t scene_id = 0;
    cv::Mat mask;
    double accuracy;
    std::string label;
    cv::Mat depth_data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_data;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_shadow;
};

struct detected_item_t
{
    uint32_t item_id = 0;
    std::shared_ptr<item_cam_t> item_cam1;
    std::shared_ptr<item_cam_t> item_cam2;
    std::string item_id_hash; //??????
    std::string label;
};

struct element_t
{
    element_t(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_2) : element_pcl_1(pcl_1),
                                                                                                            element_pcl_2(pcl_2),
                                                                                                            pcl_merged(new pcl::PointCloud<pcl::PointXYZ>),
                                                                                                            shadow_ptcld(new pcl::PointCloud<pcl::PointXYZRGB>)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cam2(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*element_pcl_1, *pcl_merged);
        pcl::copyPointCloud(*element_pcl_2, *cam2);
        *pcl_merged += *cam2;
    };
    uint32_t item_element_id;
    uint32_t item_id = 0;
    std::string label;
    std::string element_label;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr element_pcl_1;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr element_pcl_2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_merged;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr shadow_ptcld;
    std::shared_ptr<cv::Mat> element_mask_1;
    std::shared_ptr<cv::Mat> element_mask_2;
    std::shared_ptr<cv::Mat> element_depth_1;
    std::shared_ptr<cv::Mat> element_depth_2;
};

struct current_scene_t
{
    current_scene_t()
        : pcl_camera_1(new pcl::PointCloud<pcl::PointXYZ>),
          pcl_camera_2(new pcl::PointCloud<pcl::PointXYZ>),
          merged_pcl(new pcl::PointCloud<pcl::PointXYZ>){};
    uint32_t scene_id;
    cv::Mat camera_1_rgb;
    cv::Mat camera_1_depth;
    cv::Mat camera_2_rgb;
    cv::Mat camera_2_depth;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_camera_1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_camera_2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_pcl;
};

struct CameraParameters
{
    float cx;
    float cy;
    float fx;
    float fy;

    size_t width;
    size_t height;
};

struct Frames
{
    const std::string world_frame = "world";
    const std::string panda_wrist_frame = "panda_link8";
    const std::string camera_frame = "head_camera1_rgb_optical_frame";
    const std::string camera_frame2 = "head_camera2_rgb_optical_frame";
};
struct WorkspaceArea
{
    float x_min;
    float y_min;
    float z_min;

    float x_max;
    float y_max;
    float z_max;
    float camera_max_distance;
    std::string data_type;
};
#endif
