#ifndef HELPERS_VISION__CONVERTERS_HPP_
#define HELPERS_VISION__CONVERTERS_HPP_

// ___CPP___
#include <memory>
#include <chrono>

// ___OpenCV___
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

// ___PCL___
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ___Eigen___
#include <eigen3/Eigen/Eigen>

// ___JSON___
#include <nlohmann/json.hpp>

// ___ROS___
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

// ___Package___
#include "helpers_commons/timers.hpp"
#include "helpers_vision/vision.hpp"

namespace helpers
{
  namespace converters
  {
    using json = nlohmann::json;

    // ______Images______

    /**
     * Serialization of cv::Mat to std::iostream using PNG encoding
     * @param image input cv::Mat image
     * @param out_stream serialized image buffer
     * @param debug debug flag whether display time of execution
     */
    int imageToStream(const cv::Mat &image, std::shared_ptr<std::iostream> &out_stream, bool debug = false);

    /**
     * Serialization of cv::Mat to std::iostream using PNG encoding with pointers
     * @param image input cv::Mat image
     * @param out_stream serialized image buffer
     * @param debug debug flag whether display time of execution
     */
    int imageToStream(const std::shared_ptr<cv::Mat> &image, std::shared_ptr<std::iostream> &out_stream, bool debug = false);

    /**
     * Deserialization of std::iostream to cv::Mat using PNG encoding
     * @param stream input serialized image buffer
     * @param out_image output cv::Mat image
     * @param debug debug flag whether display time of execution
     */
    int streamToImage(const std::shared_ptr<std::iostream> &stream, cv::Mat &out_image, bool debug = false);

    /**
     * Deserialization of std::iostream to cv::Mat using PNG encoding with pointers
     * @param stream input serialized image buffer
     * @param out_image output cv::Mat image
     * @param debug debug flag whether display time of execution
     */
    int streamToImage(const std::shared_ptr<std::iostream> &stream, std::shared_ptr<cv::Mat> &out_image, bool debug = false);

    // ___Binary masks___

    /**
     * Serialization of cv::Mat binary mask to std::iostream using RLE encoding
     * @param mask input cv::Mat binary image
     * @param out_stream serialized image buffer
     * @param debug debug flag whether display time of execution
     */
    int binaryMaskToStream(const cv::Mat &mask, std::shared_ptr<std::iostream> &out_stream, bool debug = false);

    /**
     * Serialization of cv::Mat binary mask to std::iostream using RLE encoding with pointers
     * @param mask input cv::Mat binary image
     * @param out_stream serialized image buffer
     * @param debug debug flag whether display time of execution
     */
    int binaryMaskToStream(const std::shared_ptr<cv::Mat> &mask, std::shared_ptr<std::iostream> &out_stream, bool debug = false);

    /**
     * Serialization of cv::Mat binary mask to std::string using RLE encoding
     * @param mask input cv::Mat binary image
     * @param out_string serialized image buffer as a string
     * @param debug debug flag whether display time of execution
     */
    int binaryMaskToString(const cv::Mat &mask, std::string &out_string, bool debug = false);

    /**
     * Serialization of cv::Mat binary mask to std::strin using RLE encoding
     * @param mask pointer to input cv::Mat binary image 
     * @param out_string serialized image buffer as a string
     * @param debug debug flag whether display time of execution
     */
    int binaryMaskToString(const std::shared_ptr<cv::Mat> &mask, std::string &out_string, bool debug = false);

    /**
     * Deserialization of std::iostream to cv::Mat binary mask using RLE encoding
     * @param stream serialized binary image buffer
     * @param out_mask output cv::Mat binary image
     * @param debug debug flag whether display time of execution
     */
    int streamToBinaryMask(const std::shared_ptr<std::iostream> &stream, cv::Mat &out_mask, bool debug = false);

    /**
     * Deserialization of std::iostream to cv::Mat binary mask using RLE encoding with pointers
     * @param stream serialized binary image buffer
     * @param out_mask output cv::Mat binary image
     * @param debug debug flag whether display time of execution
     */
    int streamToBinaryMask(const std::shared_ptr<std::iostream> &stream, std::shared_ptr<cv::Mat> &out_mask, bool debug = false);

    /**
     * Deserialization of string to cv::Mat binary mask using RLE encoding
     * @param mask_str serialized binary image string
     * @param out_mask output cv::Mat binary image
     * @param debug debug flag whether display time of execution
     */
    int stringToBinaryMask(const std::string &mask_str, cv::Mat &out_mask, bool debug = false);

    /**
     * Deserialization of string to cv::Mat binary mask using RLE encoding with pointers
     * @param mask_str serialized binary image string
     * @param out_mask output cv::Mat binary image
     * @param debug debug flag whether display time of execution
     */
    int stringToBinaryMask(const std::string &mask_str, std::shared_ptr<cv::Mat> &out_mask, bool debug = false);

    /**
     * Utility function to extract pointcloud type from header.
     * @param[in] blob pointcloud on binary version
     * @return string representing type of pointcloud
     */
    std::string getPointTypes(const pcl::PCLPointCloud2 &blob);

    // ___Pointclouds___

    /**
     * Pointer version for serialization of pointcloud. 
     * @param cloud output cloud with deserialized pointcloud
     * @param out_stream stream with serialized pointcloud
     * @param debug debug flag whether display time of execution
     * @return return flag
     */
    template <typename PointT>
    int pointcloudToStream(const pcl::PointCloud<PointT> &cloud, std::shared_ptr<std::iostream> &out_stream, bool debug = false)
    {
      Timer timer("helpers::converters::pointcloudToStream", debug);
      if (cloud.points.size() == 0)
        return -1;
      pcl::PCLPointCloud2 blob;
      pcl::toPCLPointCloud2(cloud, blob);
      std::shared_ptr<std::stringstream> ss(new std::stringstream);
      (*ss) << cloud.points.size() << '\n';
      (*ss) << getPointTypes(blob) << '\n';
      ss->write(reinterpret_cast<const char *>(blob.data.data()), blob.data.size());
      out_stream = ss;
      return 0;
    }

    /**
     * Pointer version for serialization of pointcloud. 
     * @param cloud output cloud with deserialized pointcloud
     * @param out_stream stream with serialized pointcloud
     * @param debug debug flag whether display time of execution
     * @return return flag
     * Example usage:
     * helpers\::converters\::pointcloudToStream\<pcl::PointXYZ>(in_cloud, out_stream);
     */
    template <typename PointT>
    int pointcloudToStream(const typename pcl::PointCloud<PointT>::Ptr &cloud, std::shared_ptr<std::iostream> &out_stream, bool debug = false)
    {
      return pointcloudToStream(*cloud, out_stream, debug);
    }

    /**
     * Deserialization of pointcloud to std::iostream.
     * @param stream stream with serialized pointcloud
     * @param out_cloud output cloud with deserialized pointcloud
     * @param debug debug flag whether display time of execution
     * @return return flag
     */
    template <typename PointT>
    int streamToPointcloud(const std::shared_ptr<std::iostream> &stream, pcl::PointCloud<PointT> &out_cloud, bool debug = false)
    {
      Timer timer("helpers::converters::streamToPointcloud", debug);
      if (stream == nullptr)
        return -1;

      // pcl::PointXYZ is supported
      pcl::PCLPointCloud2 blob;

      std::string nr_points_str;
      std::getline(*stream, nr_points_str, '\n');
      int nr_points = 0;
      try
      {
        nr_points = std::stoi(nr_points_str);
      }
      catch (const std::invalid_argument &err)
      {
        if (debug)
          std::cerr << "helpers::converters::streamToPointcloud: Invalid argument" << std::endl;
        return -1;
      }
      blob.width = nr_points;
      blob.height = 1;
      blob.header.frame_id = "world";

      std::string point_type_str;
      std::getline(*stream, point_type_str, '\n');

      pcl::PCLPointField point_field;
      point_field.name = "x";
      point_field.datatype = pcl::PCLPointField::FLOAT32;
      point_field.count = 1;
      point_field.offset = 0;
      blob.fields.push_back(point_field);

      point_field.name = "y";
      point_field.datatype = pcl::PCLPointField::FLOAT32;
      point_field.count = 1;
      point_field.offset = 4;
      blob.fields.push_back(point_field);

      point_field.name = "z";
      point_field.datatype = pcl::PCLPointField::FLOAT32;
      point_field.count = 1;
      point_field.offset = 8;
      blob.fields.push_back(point_field);

      blob.point_step = sizeof(pcl::PointXYZ);
      blob.row_step = blob.point_step * blob.width;
      blob.data.resize(blob.point_step * nr_points);
      stream->read(reinterpret_cast<char *>(&blob.data[0]), blob.data.size());
      pcl::fromPCLPointCloud2(blob, out_cloud);

      stream->clear();
      stream->seekg(0);
      return 0;
    }

    /**
     * Pointer version for deserialization of pointcloud. 
     * @param stream stream with serialized pointcloud
     * @param out_cloud output cloud with deserialized pointcloud
     * @param debug debug flag whether display time of execution
     * @return return flag
     * Example usage:
     * helpers\::converters\::streamToPointcloud\<pcl::PointXYZ>(stream, out_cloud);
     */
    template <typename PointT>
    int streamToPointcloud(const std::shared_ptr<std::iostream> &stream, typename pcl::PointCloud<PointT>::Ptr &out_cloud, bool debug = false)
    {
      return streamToPointcloud(stream, *out_cloud, debug);
    }

    //TODO doxygen
    int rosImageToCV(const sensor_msgs::msg::Image &ros_image, cv::Mat &out_image);
    int cvMatToRos(const cv::Mat &cv_image, sensor_msgs::msg::Image &out_ros_image);
    template <typename PointT>
    int rosPtcldtoPcl(const sensor_msgs::msg::PointCloud2 &ros_cloud_msg, typename pcl::PointCloud<PointT>::Ptr &out_pcl_cloud)
    {
      if (out_pcl_cloud == nullptr)
        out_pcl_cloud = vision::makeSharedPcl<PointT>();
      if (ros_cloud_msg.fields.size() == 0)
        return -1;
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(ros_cloud_msg, pcl_pc2);
      pcl::fromPCLPointCloud2(pcl_pc2, *out_pcl_cloud);
      out_pcl_cloud->header.frame_id = ros_cloud_msg.header.frame_id;
      return 0;
    }
    template <typename PointT>
    int pclToRosPtcld(typename pcl::PointCloud<PointT>::Ptr pcl_cloud, sensor_msgs::msg::PointCloud2 &out_ros_cloud)
    {
      pcl::toROSMsg(*pcl_cloud, out_ros_cloud);
      out_ros_cloud.header.frame_id = pcl_cloud->header.frame_id;
      return 0;
    }
    int eigenAffineToGeometry(const Eigen::Affine3f &eigen_aff, geometry_msgs::msg::Pose &out_ros_pose);
    int eigenAffineToGeometry(const Eigen::Affine3f &eigen_aff, geometry_msgs::msg::Transform &out_ros_pose);

    int geometryToEigenAffine(const geometry_msgs::msg::Transform &ros_pose, Eigen::Affine3f &out_eigen_aff);
    int geometryToEigenAffine(const geometry_msgs::msg::Pose &ros_pose, Eigen::Affine3f &out_eigen_aff);

    int iostreamToString(std::shared_ptr<std::iostream> &stream, std::string &out_string);

  } // namespace converters
} // namespace helpers

#endif