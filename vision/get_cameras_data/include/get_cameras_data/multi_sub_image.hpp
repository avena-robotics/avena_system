
#include <tuple>
#include <cstdarg>
#include <iostream>
// ___ROS2___
#include <rclcpp/rclcpp.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// ___Avena___
#include "custom_interfaces/msg/images.hpp"
#include "helpers_vision/helpers_vision.hpp"
#include "helpers_commons/helpers_commons.hpp"

#define SYNCHRO(x) message_filters::Synchronizer < ApproximateTime <x> > 
#define PTR(x) std::shared_ptr<x>

#define IMAGE_2 sensor_msgs::msg::Image, sensor_msgs::msg::Image
#define IMAGE_3 IMAGE_2, sensor_msgs::msg::Image
#define IMAGE_4 IMAGE_3, sensor_msgs::msg::Image
#define IMAGE_5 IMAGE_4, sensor_msgs::msg::Image
#define IMAGE_6 IMAGE_5, sensor_msgs::msg::Image
#define IMAGE_7 IMAGE_6, sensor_msgs::msg::Image
#define IMAGE_8 IMAGE_7, sensor_msgs::msg::Image
#define IMAGE_9 IMAGE_8, sensor_msgs::msg::Image

#define SUBS_2 *(*img_sumbs)[0], *(*img_sumbs)[1]
#define SUBS_3 SUBS_2, *(*img_sumbs)[2]
#define SUBS_4 SUBS_3, *(*img_sumbs)[3]
#define SUBS_5 SUBS_4, *(*img_sumbs)[4]
#define SUBS_6 SUBS_5, *(*img_sumbs)[5]
#define SUBS_7 SUBS_6, *(*img_sumbs)[6]
#define SUBS_8 SUBS_7, *(*img_sumbs)[7]
#define SUBS_9 SUBS_8, *(*img_sumbs)[8]

#define PLACEHOLDERS_2 std::placeholders::_1, std::placeholders::_2
#define PLACEHOLDERS_3 PLACEHOLDERS_2, std::placeholders::_3
#define PLACEHOLDERS_4 PLACEHOLDERS_3, std::placeholders::_4
#define PLACEHOLDERS_5 PLACEHOLDERS_4, std::placeholders::_5
#define PLACEHOLDERS_6 PLACEHOLDERS_5, std::placeholders::_6
#define PLACEHOLDERS_7 PLACEHOLDERS_6, std::placeholders::_7
#define PLACEHOLDERS_8 PLACEHOLDERS_7, std::placeholders::_8
#define PLACEHOLDERS_9 PLACEHOLDERS_8, std::placeholders::_9

#define IMG_MSG_2 img_msg, img_msg
#define IMG_MSG_3 IMG_MSG_2, img_msg
#define IMG_MSG_4 IMG_MSG_3, img_msg
#define IMG_MSG_5 IMG_MSG_4, img_msg
#define IMG_MSG_6 IMG_MSG_5, img_msg
#define IMG_MSG_7 IMG_MSG_6, img_msg
#define IMG_MSG_8 IMG_MSG_7, img_msg
#define IMG_MSG_9 IMG_MSG_8, img_msg


namespace synchronizers_image
{
  using namespace message_filters;
  using namespace sync_policies;

  using image_subscriptions = std::vector<PTR(message_filters::Subscriber<sensor_msgs::msg::Image>)>;

  using cam2 = SYNCHRO(IMAGE_2);
  using cam3 = SYNCHRO(IMAGE_3);
  using cam4 = SYNCHRO(IMAGE_4);
  using cam5 = SYNCHRO(IMAGE_5);
  using cam6 = SYNCHRO(IMAGE_6);
  using cam7 = SYNCHRO(IMAGE_7);
  using cam8 = SYNCHRO(IMAGE_8);
  using cam9 = SYNCHRO(IMAGE_9);

  using img_msg = sensor_msgs::msg::Image::ConstSharedPtr;
  using cam_tup = std::tuple<PTR(cam2), PTR(cam3), PTR(cam4), PTR(cam5), PTR(cam6), PTR(cam7), PTR(cam8), PTR(cam9)>;

  class Images
  {
  public:
    Images(size_t cam_amount, rclcpp::Node *node, image_subscriptions *img_sumbs, std::string pub_topic_name) : _cam_amount(cam_amount), _node(node)
    {

      switch (cam_amount)
      {
      case 1:
        (*img_sumbs)[0]->registerCallback([this](img_msg msg)
                                          { _synchronizedTopicsCallback(msg); });
        break;
      case 2:
        std::get<PTR(cam2)>(tup) = std::make_shared<cam2>(2, SUBS_2);
        break;
      case 3:
        std::get<PTR(cam3)>(tup) = std::make_shared<cam3>(3, SUBS_3);
        break;
      case 4:
        std::get<PTR(cam4)>(tup) = std::make_shared<cam4>(4, SUBS_4);
        break;
      case 5:
        std::get<PTR(cam5)>(tup) = std::make_shared<cam5>(5, SUBS_5);
        break;
      case 6:
        std::get<PTR(cam6)>(tup) = std::make_shared<cam6>(6, SUBS_6);
        break;
      case 7:
        std::get<PTR(cam7)>(tup) = std::make_shared<cam7>(7, SUBS_7);
        break;
      case 8:
        std::get<PTR(cam8)>(tup) = std::make_shared<cam8>(8, SUBS_8);
        break;
      case 9:
        std::get<PTR(cam9)>(tup) = std::make_shared<cam9>(9, SUBS_9);
        break;
      default:
        return;
      }

      rclcpp::QoS qos_setting = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable();
      _images_pub = _node->create_publisher<custom_interfaces::msg::Images>(pub_topic_name, qos_setting);
      _registerCallback();
    }

  private:
    void _registerCallback()
    {

      switch (_cam_amount)
      {
      case 2:
        std::get<PTR(cam2)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_2>, this, PLACEHOLDERS_2));
        break;
      case 3:
        std::get<PTR(cam3)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_3>, this, PLACEHOLDERS_3));
        break;
      case 4:
        std::get<PTR(cam4)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_4>, this, PLACEHOLDERS_4));
        break;
      case 5:
        std::get<PTR(cam5)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_5>, this, PLACEHOLDERS_5));
        break;
      case 6:
        std::get<PTR(cam6)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_6>, this, PLACEHOLDERS_6));
        break;
      case 7:
        std::get<PTR(cam7)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_7>, this, PLACEHOLDERS_7));
        break;
      case 8:
        std::get<PTR(cam8)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_8>, this, PLACEHOLDERS_8));
        break;
      case 9:
        std::get<PTR(cam9)>(tup)->registerCallback(std::bind(&Images::_synchronizedTopicsCallback<IMG_MSG_9>, this, PLACEHOLDERS_9));
        break;
      default:
        return;
      }
    }

    void _synchronizedTopicsCallback(img_msg arg)
    {
      helpers::Timer timer("publishing took:", _node->get_logger());

      _lock = true;
      custom_interfaces::msg::Images data;
      _data_vec.push_back(arg);
      for (auto img : _data_vec)
        data.data.push_back(*img);

      _data_vec.clear();

      data.header.stamp = _node->now();
      _images_pub->publish(data);
      _lock = false;
    }

    template <typename T, typename... Types>
    void _synchronizedTopicsCallback(T var1, Types... var2)
    {

      if (!_lock)
      {
        _data_vec.push_back(var1);
        _synchronizedTopicsCallback(var2...);
      }
    }

    size_t _cam_amount;
    cam_tup tup;
    bool _lock = false;
    rclcpp::Node *_node;


    std::vector<img_msg> _data_vec;
    rclcpp::Publisher<custom_interfaces::msg::Images>::SharedPtr _images_pub;
  };
}

#undef SYNCHRO
#undef PTR
#undef IMAGE_2
#undef IMAGE_3
#undef IMAGE_4
#undef IMAGE_5
#undef IMAGE_6
#undef IMAGE_7
#undef IMAGE_8
#undef IMAGE_9
#undef SUBS_2
#undef SUBS_3
#undef SUBS_4
#undef SUBS_5
#undef SUBS_6
#undef SUBS_7
#undef SUBS_8
#undef SUBS_9
#undef PLACEHOLDERS_2
#undef PLACEHOLDERS_3
#undef PLACEHOLDERS_4
#undef PLACEHOLDERS_5
#undef PLACEHOLDERS_6
#undef PLACEHOLDERS_7
#undef PLACEHOLDERS_8
#undef PLACEHOLDERS_9
#undef IMG_MSG_2
#undef IMG_MSG_3
#undef IMG_MSG_4
#undef IMG_MSG_5
#undef IMG_MSG_6
#undef IMG_MSG_7
#undef IMG_MSG_8
#undef IMG_MSG_9