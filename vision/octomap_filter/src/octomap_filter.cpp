#include "octomap_filter.hpp"

namespace octomap_filter
{
  OctomapFilter::OctomapFilter(const rclcpp::NodeOptions &options)
      : Node("octomap_filter", options)
  // _merged_point_cloud(new pcl::PointCloud<pcl::PointXYZ>),
  // _octomap_data_filtered(new pcl::PointCloud<pcl::PointXYZ>)

  {
    helpers::commons::setLoggerLevelFromParameter(this);

    RCLCPP_INFO(get_logger(), "Initialization of octomap filter.");
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)); //.transient_local().reliable();
    _initializeSubscribers(qos_settings);
    _publisher_octomap_filter = create_publisher<custom_interfaces::msg::FilteredSceneOctomap>("octomap_filter", qos_settings);
    _action_server = rclcpp_action::create_server<OctomapFilterAction>(
        this,
        "octomap_filter",
        std::bind(&OctomapFilter::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&OctomapFilter::_handleCancel, this, std::placeholders::_1),
        std::bind(&OctomapFilter::_handleAccepted, this, std::placeholders::_1));
    _readOctomapVoxelSizeFromParametersServer(_out_labels_parameters);

    _merged_ptcld_filtered_msg = std::make_shared<custom_interfaces::msg::MergedPtcldFiltered>();
    _items_msg = std::make_shared<custom_interfaces::msg::Items>();

    _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    status = custom_interfaces::msg::Heartbeat::STOPPED;

    RCLCPP_INFO(get_logger(), "...octomap filter init done .");
  }

  void OctomapFilter::initNode()
  {
    status = custom_interfaces::msg::Heartbeat::STARTING;

    status = custom_interfaces::msg::Heartbeat::RUNNING;
  }

  void OctomapFilter::shutDownNode()
  {
    status = custom_interfaces::msg::Heartbeat::STOPPING;

    status = custom_interfaces::msg::Heartbeat::STOPPED;
  }

  void OctomapFilter::_initializeSubscribers(const rclcpp::QoS &qos_settings)
  {
    _subscriber_merged_items = create_subscription<custom_interfaces::msg::Items>(
        "merged_items", qos_settings,
        [this](custom_interfaces::msg::Items::SharedPtr items_msg)
        {
          RCLCPP_DEBUG(get_logger(), "Merged items message received.");
          _items_msg = items_msg;
        });

    _subscriber_merged_ptcld_filtered = create_subscription<custom_interfaces::msg::MergedPtcldFiltered>(
        "merged_ptcld_filtered", qos_settings,
        [this](custom_interfaces::msg::MergedPtcldFiltered::SharedPtr merged_ptcld_filtered_msg)
        {
          RCLCPP_DEBUG(get_logger(), "Merged point cloud filtered message received.");
          _merged_ptcld_filtered_msg = merged_ptcld_filtered_msg;
        });
  }

  rclcpp_action::GoalResponse OctomapFilter::_handleGoal(const rclcpp_action::GoalUUID &, std::shared_ptr<const OctomapFilterAction::Goal>)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with order ");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  rclcpp_action::CancelResponse OctomapFilter::_handleCancel(const std::shared_ptr<GoalHandleOctomapFilter> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }
  void OctomapFilter::_handleAccepted(const std::shared_ptr<GoalHandleOctomapFilter> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&OctomapFilter::_execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void OctomapFilter::_execute(const std::shared_ptr<GoalHandleOctomapFilter> goal_handle)
  {
    helpers::Timer timer("Octomap filter action", get_logger());
    auto result = std::make_shared<OctomapFilterAction::Result>();
    if (status != custom_interfaces::msg::Heartbeat::RUNNING)
    {
      RCLCPP_WARN_ONCE(this->get_logger(), "Node is not in running state");
      goal_handle->abort(result);
      return;
    }
    RCLCPP_INFO(get_logger(), "Executing goal");

    auto abort_goal = [this, goal_handle, result](std::string error_message) -> void
    {
      RCLCPP_ERROR(get_logger(), error_message);
      _publisher_octomap_filter->publish(custom_interfaces::msg::FilteredSceneOctomap());
      goal_handle->abort(result);
    };

    if (_validateInput())
    {
      abort_goal("Invalid input data. Goal failed");
      return;
    }
    _checkLastMessagesTimestamps(_items_msg->header.stamp, _merged_ptcld_filtered_msg->header.stamp);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    if (_voxelizePointcloud(scene_filtered))
    {
      abort_goal("Error occured while voxelizing.");
      return;
    }

    // _input_msg_data= *_input_msg_data;
    custom_interfaces::msg::FilteredSceneOctomap::UniquePtr octomap_filter_msg = _prepareOutputMsg(scene_filtered);
    if (!octomap_filter_msg)
    {
      abort_goal("Error occured while preparing output message.");
      return;
    }

    if (rclcpp::ok())
    {
      _publisher_octomap_filter->publish(std::move(octomap_filter_msg));
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal succeeded.");
    }
    else
      throw std::runtime_error("Problem with ROS");
  }

  void OctomapFilter::_getItemPtcld(pcl::PointCloud<pcl::PointXYZ>::Ptr item_pointcloud)
  {
    // auto item_result_iter = std::find_if(_input_msg_data.items.begin(), _item_select_data.items.end(),
    //                                      [this, selected_item_id](custom_interfaces::msg::Item item) {
    //                                        return item.id == selected_item_id;
    //                                      });
    // pcl::PointCloud<pcl::PointXYZ>::Ptr item_ptcld_vis(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr shadow_ptcld_vis(new pcl::PointCloud<pcl::PointXYZ>);

    for (auto &item : _items_msg->items)
    {
      for (auto &item_element : item.item_elements)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr item_element_ptcld(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr item_element_shadow_ptcld(new pcl::PointCloud<pcl::PointXYZ>);

        helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(item_element.merged_ptcld, item_element_ptcld);
        helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(item_element.shadow_ptcld, item_element_shadow_ptcld);
        *item_pointcloud += *item_element_ptcld;
        *item_pointcloud += *item_element_shadow_ptcld;

        // *item_ptcld_vis += *item_element_ptcld;
        // *shadow_ptcld_vis += *item_element_shadow_ptcld;
      }
    }

    // std::cout << "item_ptcld_vis->points.size()" << item_ptcld_vis->points.size() << std::endl;
    // std::cout << "shadow_ptcld_vis->points.size()" << shadow_ptcld_vis->points.size() << std::endl;

    // pcl::visualization::PCLVisualizer viewer;

    // viewer.addPointCloud<pcl::PointXYZ>(shadow_ptcld_vis, "shadow");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "shadow");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, "shadow");

    // viewer.addPointCloud<pcl::PointXYZ>(item_ptcld_vis, "filtered_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "filtered_cloud");
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "filtered_cloud");
    // // viewer.addCoordinateSystem(0.1, transform, "Affine_transform");
    // viewer.addCoordinateSystem(0.1);
    // viewer.spin();
    // std::cout << "Visualize" << std::endl;

    // helpers::vision::passThroughFilter(item_pointcloud, "z", -0.005, 0.005, true);
  }

  int OctomapFilter::_cropBox(const Eigen::Affine3f &transform, const Eigen::Vector3f dimensions, bool outside_of_the_box, pcl::PointCloud<pcl::PointXYZ>::Ptr &ptcld)
  {
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setInputCloud(ptcld);
    crop_box.setMin(Eigen::Vector4f(-dimensions.x() / 2, -dimensions.y() / 2, -dimensions.z() / 2, 1.0));
    crop_box.setMax(Eigen::Vector4f(+dimensions.x() / 2, +dimensions.y() / 2, +dimensions.z() / 2, 1.0));
    crop_box.setTransform(transform.inverse());
    crop_box.setNegative(outside_of_the_box);
    crop_box.filter(*ptcld);
    return 0;
  }

  bool OctomapFilter::_voxelizePointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr out_scene_filtered)
  {
    // _octomap_data_filtered->clear();
    pcl::PointCloud<pcl::PointXYZ>::Ptr buffer(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_merged_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(_merged_ptcld_filtered_msg->merged_ptcld_filtered, scene_merged_filtered);
    if (scene_merged_filtered->size() == 0)
    {
      RCLCPP_INFO(get_logger(), "Point cloud of the scene is empty");
      return true;
    }

    // std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> scene_voxels;
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> scene_voxels_occ;
    // remove table from scene
    helpers::vision::passThroughFilter(scene_merged_filtered, "z", 0.005, 2.0f, false);

    DishData bowl_data;
    DishData plate_data;
    KnifeData knife_data;
    SpatulaData spatula_data;
    for (auto label : _out_labels_parameters)
    {
      if (label.label == "bowl")
      {
        // std::cout << "label.item_description.dump():: " << label.item_description.dump() << std::endl;
        bowl_data.alpha = label.item_description["lower_angle"].get<float>();
        bowl_data.beta = label.item_description["higher_angle"].get<float>();
        bowl_data.lower_disc_height = label.item_description["lower_disc_height"].get<float>();
        bowl_data.lower_disc_radius = label.item_description["lower_disc_radius"].get<float>() + 0.005; // offset for removing the pointcloud
        bowl_data.middle_disc_height = label.item_description["middle_disc_height"].get<float>();
        bowl_data.middle_disc_radius = label.item_description["middle_disc_radius"].get<float>();
        bowl_data.upper_disc_height = label.item_description["upper_disc_height"].get<float>();
        bowl_data.upper_disc_radius = label.item_description["upper_disc_radius"].get<float>();
        bowl_data.base_height = label.item_description["base_height"].get<float>();
      }
      else if (label.label == "plate")
      {
        // std::cout << "label.item_description.dump():: " << label.item_description.dump() << std::endl;
        plate_data.alpha = label.item_description["lower_angle"].get<float>();
        plate_data.beta = label.item_description["higher_angle"].get<float>();
        plate_data.lower_disc_height = label.item_description["lower_disc_height"].get<float>();
        plate_data.lower_disc_radius = label.item_description["lower_disc_radius"].get<float>() + 0.02; // offset for removing the pointcloud
        plate_data.middle_disc_height = label.item_description["middle_disc_height"].get<float>();
        plate_data.middle_disc_radius = label.item_description["middle_disc_radius"].get<float>();
        plate_data.upper_disc_height = label.item_description["upper_disc_height"].get<float>();
        plate_data.upper_disc_radius = label.item_description["upper_disc_radius"].get<float>();
        plate_data.base_height = label.item_description["base_height"].get<float>();
      }
      else if (label.label == "knife")
      {
        // std::cout << "label.item_description.dump():: " << label.item_description.dump() << std::endl;
        knife_data.offset_blade = helpers::vision::assignPositionFromJson(label.item_description["offset_blade"]);
        knife_data.dimensions_blade = helpers::vision::assignPositionFromJson(label.item_description["dimensions_blade"]);
        knife_data.offset_handle = helpers::vision::assignPositionFromJson(label.item_description["offset_handle"]);
        knife_data.dimensions_handle = helpers::vision::assignPositionFromJson(label.item_description["dimensions_handle"]);
      }
      else if (label.label == "spatula")
      {
        // std::cout << "label.item_description.dump():: " << label.item_description.dump() << std::endl;
        spatula_data.offset_handle = helpers::vision::assignPositionFromJson(label.item_description["offset_handle"]);
        spatula_data.dimensions_handle = helpers::vision::assignPositionFromJson(label.item_description["dimensions_handle"]);
        spatula_data.offset_connector = helpers::vision::assignPositionFromJson(label.item_description["offset_connector"]);
        spatula_data.dimensions_connector = helpers::vision::assignPositionFromJson(label.item_description["dimensions_connector"]);
        spatula_data.offset_spatula_tool = helpers::vision::assignPositionFromJson(label.item_description["offset_spatula_tool"]);
        spatula_data.dimensions_spatula_tool = helpers::vision::assignPositionFromJson(label.item_description["dimensions_spatula_tool"]);
      }
    }

    for (auto item : _items_msg->items)
    {
      for (auto item_element : item.item_elements)
      {

        for (auto part : item_element.parts_description)
        {
          json data;
          data = json::parse(part);
          // RCLCPP_INFO(get_logger(), "Part description :  %s", data.dump().c_str());
          if (data.contains("fit_method"))
          {
            if (data["fit_method"] == "box")
            {
              float dims_buffor = 1.1;
              Eigen::Vector3f dimensions = helpers::vision::assignPositionFromJson(data["dims"]);
              Eigen::Vector3f position = helpers::vision::assignPositionFromJson(data["pose"]["position"]);
              Eigen::Quaternionf orientation = helpers::vision::assignQuaternionFromJson(data["pose"]["orientation"]);
              Eigen::Affine3f item_transform;
              helpers::converters::geometryToEigenAffine(item.pose, item_transform);
              // calculate part each part affine transform
              Eigen::Affine3f part_transform(Eigen::Affine3f::Identity());
              part_transform = item_transform * (Eigen::Translation3f(position) * orientation);
              // safety buffor for obcject size
              dimensions = dimensions * dims_buffor;

              _cropBox(part_transform, dimensions, true, scene_merged_filtered);

              // helpers::vision::visualize({scene_merged_filtered}, {part_transform});
            }
            else if (data["fit_method"] == "sphere")
            {
              float radius_buffer = 0.01; //1 cm

              float radius = data["dims"]["radius"].get<float>();
              Eigen::Vector3f position = helpers::vision::assignPositionFromJson(data["pose"]["position"]);
              Eigen::Quaternionf orientation = helpers::vision::assignQuaternionFromJson(data["pose"]["orientation"]);
              Eigen::Affine3f item_transform;
              helpers::converters::geometryToEigenAffine(item.pose, item_transform);
              // calculate part each part affine transform
              Eigen::Affine3f part_transform(Eigen::Affine3f::Identity());
              part_transform = item_transform * (Eigen::Translation3f(position) * orientation);

              pcl::ModelCoefficients model_coeff;
              model_coeff.values.resize(4);
              model_coeff.values[0] = part_transform.translation().x();
              model_coeff.values[1] = part_transform.translation().y();
              model_coeff.values[2] = part_transform.translation().z();
              model_coeff.values[3] = radius + radius_buffer;

              pcl::ModelOutlierRemoval<pcl::PointXYZ> sphere_filter;
              sphere_filter.setModelCoefficients(model_coeff);
              sphere_filter.setInputCloud(scene_merged_filtered);
              sphere_filter.setModelType(pcl::SACMODEL_SPHERE);
              sphere_filter.setThreshold(0.005); //5mm
              sphere_filter.setNegative(true);
              sphere_filter.filter(*scene_merged_filtered);

              // helpers::vision::visualize({scene_merged_filtered}, {part_transform});
            }
            else if (data["fit_method"] == "cylinder")
            {
              float radius_buffer = 0.01;

              float radius = data["dims"]["radius"].get<float>();
              float height = data["dims"]["height"].get<float>();
              Eigen::Vector3f position = helpers::vision::assignPositionFromJson(data["pose"]["position"]);
              // define cylinder both ensd
              Eigen::Vector3f position_min(position.x(), position.y(), position.z() - height / 2);
              Eigen::Vector3f position_max(position.x(), position.y(), position.z() + height / 2);

              Eigen::Quaternionf orientation = helpers::vision::assignQuaternionFromJson(data["pose"]["orientation"]);
              Eigen::Affine3f item_transform;
              helpers::converters::geometryToEigenAffine(item.pose, item_transform);
              // calculate part each part affine transform
              Eigen::Affine3f part_transform(Eigen::Affine3f::Identity());
              part_transform = item_transform * (Eigen::Translation3f(position) * orientation.toRotationMatrix());
              position_min = part_transform.rotation() * position_min + part_transform.translation();
              position_max = part_transform.rotation() * position_max + part_transform.translation();
              pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
              {
                // helpers::Timer timer("cylinder erase", true);
                for (pcl::PointCloud<pcl::PointXYZ>::iterator it = scene_merged_filtered->begin(); it != scene_merged_filtered->end(); /*it++*/)
                {
                  Eigen::Vector3f point(it->x, it->y, it->z);
                  float distance;
                  _getDistanceFromLine(point, position_min, position_max, distance);
                  if (distance < radius + radius_buffer)
                  {
                    it = scene_merged_filtered->erase(it);
                  }
                  else
                    it++;
                }
              }
            }
            else if (data["fit_method"] == "knife")
            {
              float dims_buffor = 1.3;
              Eigen::Vector3f position = helpers::vision::assignPositionFromJson(data["pose"]["position"]);
              Eigen::Quaternionf orientation = helpers::vision::assignQuaternionFromJson(data["pose"]["orientation"]);
              Eigen::Affine3f item_transform;
              helpers::converters::geometryToEigenAffine(item.pose, item_transform);

              Eigen::Vector3f position_handle = position + knife_data.offset_handle;
              Eigen::Affine3f handle_transform(Eigen::Affine3f::Identity());
              handle_transform = item_transform * (Eigen::Translation3f(position_handle) * orientation);
              _cropBox(handle_transform, knife_data.dimensions_handle * dims_buffor, true, scene_merged_filtered);

              Eigen::Vector3f position_blade = position + knife_data.offset_blade;
              Eigen::Affine3f blade_transform(Eigen::Affine3f::Identity());
              blade_transform = item_transform * (Eigen::Translation3f(position_blade) * orientation);
              _cropBox(blade_transform, knife_data.dimensions_blade * dims_buffor, true, scene_merged_filtered);
              // helpers::vision::visualize({scene_merged_filtered}, {handle_transform, blade_transform});
            }
            else if (data["fit_method"] == "spatula")
            {
              float dims_buffor = 1.3;

              Eigen::Vector3f position = helpers::vision::assignPositionFromJson(data["pose"]["position"]);
              Eigen::Quaternionf orientation = helpers::vision::assignQuaternionFromJson(data["pose"]["orientation"]);
              Eigen::Affine3f item_transform;
              helpers::converters::geometryToEigenAffine(item.pose, item_transform);
              // handle
              Eigen::Vector3f position_handle = position + spatula_data.offset_handle;
              Eigen::Affine3f handle_transform(Eigen::Affine3f::Identity());
              handle_transform = item_transform * (Eigen::Translation3f(position_handle) * orientation);
              _cropBox(handle_transform, spatula_data.dimensions_handle * dims_buffor, true, scene_merged_filtered);
              // connector
              Eigen::Vector3f position_spatula_connector = position + spatula_data.offset_connector;
              Eigen::Affine3f spatula_connector_transform(Eigen::Affine3f::Identity());
              spatula_connector_transform = item_transform * (Eigen::Translation3f(position_spatula_connector) * orientation);
              _cropBox(spatula_connector_transform, spatula_data.dimensions_connector * dims_buffor, true, scene_merged_filtered);
              // spatula tool
              Eigen::Vector3f position_spatula_tool = position + spatula_data.offset_spatula_tool;
              Eigen::Affine3f spatula_tool_transform(Eigen::Affine3f::Identity());
              spatula_tool_transform = item_transform * (Eigen::Translation3f(position_spatula_tool) * orientation);
              _cropBox(spatula_tool_transform, spatula_data.dimensions_spatula_tool * dims_buffor, true, scene_merged_filtered);

              // helpers::vision::visualize({scene_merged_filtered}, {handle_transform, spatula_connector_transform, handle_transform, spatula_tool_transform});
            }

            else if (data["fit_method"] == "bowl")
            {
              float buffor_radius = 0.015;
              float buffor_height = 0.01;

              Eigen::Affine3f item_transform;
              helpers::converters::geometryToEigenAffine(item.pose, item_transform);
              _filterDish(bowl_data, data, item_transform, buffor_radius, buffor_height, scene_merged_filtered);
            }
            else if (data["fit_method"] == "plate")
            {
              float buffor_radius = 0.01;
              float buffor_height = 0.015;

              Eigen::Affine3f item_transform;
              helpers::converters::geometryToEigenAffine(item.pose, item_transform);
              _filterDish(plate_data, data, item_transform, buffor_radius, buffor_height, scene_merged_filtered);
            }
          }
          else
          {
            RCLCPP_INFO(get_logger(), "part description is empty ");
          }
        }
      }
    }

    _getItemPtcld(buffer);
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree_occupancy(_grid_size);
    octree_occupancy.setInputCloud(scene_merged_filtered);
    octree_occupancy.addPointsFromInputCloud();
    float radius_search = _grid_size * 5;

    // execute octree radius search
    for (const auto &point : buffer->points)
    {
      std::vector<int> cloudNWRSearch;
      std::vector<float> cloudNWRRadius;
      octree_occupancy.radiusSearch(point, radius_search, cloudNWRSearch, cloudNWRRadius);
      for (const auto &cloud_ix : cloudNWRSearch)
      {
        octree_occupancy.deleteVoxelAtPoint(cloud_ix);
      }
    }

    octree_occupancy.getOccupiedVoxelCenters(scene_voxels_occ);

    out_scene_filtered->points.resize(scene_voxels_occ.size());
    for (size_t i = 0; i < scene_voxels_occ.size(); i++)
      out_scene_filtered->points[i].getVector3fMap() = scene_voxels_occ[i].getVector3fMap();
    return false;
  }

  int OctomapFilter::_filterDish(DishData dish_data, json data, Eigen::Affine3f item_transform, float buffor_radius, float buffor_height, pcl::PointCloud<pcl::PointXYZ>::Ptr &scene_merged_filtered)
  {
    Eigen::Vector3f position = helpers::vision::assignPositionFromJson(data["pose"]["position"]);
    position.z() = position.z() - dish_data.upper_disc_height / 2;
    // define cylinder both ensd
    Eigen::Vector3f position_min(position.x(), position.y(), position.z());
    Eigen::Vector3f position_max(position.x(), position.y(), position.z() + dish_data.upper_disc_height);

    Eigen::Quaternionf orientation = helpers::vision::assignQuaternionFromJson(data["pose"]["orientation"]);
    // calculate part each part affine transform
    Eigen::Affine3f part_transform(Eigen::Affine3f::Identity());
    Eigen::Affine3f part_transform_min(Eigen::Affine3f::Identity());
    Eigen::Affine3f part_transform_max(Eigen::Affine3f::Identity());

    part_transform = item_transform * (Eigen::Translation3f(position) * orientation.toRotationMatrix());
    part_transform_min = item_transform * (Eigen::Translation3f(position_min) * orientation.toRotationMatrix());
    part_transform_max = item_transform * (Eigen::Translation3f(position_max) * orientation.toRotationMatrix());

    position_min = part_transform.rotation() * position_min + part_transform.translation();
    position_max = part_transform.rotation() * position_max + part_transform.translation();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr base(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr middle(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr upper(new pcl::PointCloud<pcl::PointXYZ>);
    {
      // helpers::Timer timer("erase time", true);
      for (pcl::PointCloud<pcl::PointXYZ>::iterator it = scene_merged_filtered->begin(); it != scene_merged_filtered->end(); /*it++*/)
      {

        Eigen::Vector3f point_local(it->x, it->y, it->z);
        point_local = part_transform.inverse() * point_local;
        Eigen::Vector3f point(it->x, it->y, it->z);
        if (-buffor_height < point_local.z() && point_local.z() < dish_data.base_height)
        {
          float distance;
          _getDistanceFromLine(point, part_transform_min.translation(), part_transform_max.translation(), distance);
          if (distance < dish_data.lower_disc_radius + buffor_radius)
          {
            // pcl::PointXYZ point_(it->x, it->y, it->z);
            // base->push_back(point_);
            it = scene_merged_filtered->erase(it);
          }
          else
            it++;
        }
        else if (dish_data.base_height <= point_local.z() && point_local.z() < dish_data.middle_disc_height)
        {
          float distance;
          _getDistanceFromLine(point, part_transform_min.translation(), part_transform_max.translation(), distance);
          float radius_correction = std::tan(dish_data.alpha * M_PI) * std::abs(point_local.z() - dish_data.lower_disc_height);
          if ((dish_data.lower_disc_radius + buffor_radius + radius_correction) > distance && (distance < (dish_data.lower_disc_radius - buffor_radius + radius_correction)))
          {
            // pcl::PointXYZ point_(it->x, it->y, it->z);
            // middle->push_back(point_);
            it = scene_merged_filtered->erase(it);
          }
          else
            it++;
        }
        else if (dish_data.middle_disc_height <= point_local.z() && point_local.z() < dish_data.upper_disc_height + buffor_height)
        {
          float distance;
          _getDistanceFromLine(point, part_transform_min.translation(), part_transform_max.translation(), distance);
          float radius_correction = std::tan(dish_data.beta * M_PI) * std::abs(point_local.z() - dish_data.middle_disc_height);
          if ((dish_data.middle_disc_radius - buffor_radius + radius_correction) < distance && distance < (dish_data.upper_disc_radius + buffor_radius + radius_correction))
          {
            // pcl::PointXYZ point_(it->x, it->y, it->z);
            // upper->push_back(point_);
            it = scene_merged_filtered->erase(it);
          }
          else
            it++;
        }
        else
        {
          it++;
        }
      }
    }

    // helpers::vision::visualize({scene_merged_filtered, base, middle, upper}, {part_transform, part_transform_min, part_transform_max});
    return 0;
  }

  void OctomapFilter::_getDistanceFromLine(Eigen::Vector3f point, Eigen::Vector3f line_origin, Eigen::Vector3f line_direction, float &distance)
  {

    distance = sqrt(pcl::sqrPointToLineDistance(Eigen::Vector4f(point.x(), point.y(), point.z(), 0), Eigen::Vector4f(line_origin.x(), line_origin.y(), line_origin.z(), 0),
                                                Eigen::Vector4f(line_direction.x() - line_origin.x(), line_direction.y() - line_origin.y(), line_direction.z() - line_origin.z(), 0)));
  }
  void OctomapFilter::_readOctomapVoxelSizeFromParametersServer(std::vector<Label> &out_labels_parameters)
  {
    try
    {
      json area = helpers::commons::getParameter("octomap_filter");
      _grid_size = area["octomap_voxel_size"].get<float>();
    }
    catch (const json::exception &e)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
      throw std::runtime_error("Get occupancy grid reading grid size from JSON error.");
    }
    try
    {
      json labels_params = helpers::commons::getParameter("labels");
      for (auto label_params : labels_params)
      {
        Label label_parameters;
        label_parameters.label = label_params["label"].get<std::string>();
        label_parameters.fit_method = label_params["fit_method"].get<std::string>();
        label_parameters.fit_method_parameters = label_params["fit_method_parameters"];
        label_parameters.components = label_params["components"];
        label_parameters.item = label_params["item"].get<bool>();
        label_parameters.element = label_params["element"].get<bool>();
        if (label_params.contains("item_description"))
          label_parameters.item_description = label_params["item_description"];
        out_labels_parameters.push_back(label_parameters);
      }
    }
    catch (const json::exception &e)
    {
      RCLCPP_FATAL_STREAM(get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
      throw std::runtime_error("Error occured while parsing \"labels\" from parameter server");
    }
  }

  custom_interfaces::msg::FilteredSceneOctomap::UniquePtr OctomapFilter::_prepareOutputMsg(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered)
  {

    custom_interfaces::msg::FilteredSceneOctomap::UniquePtr octomap_filter_msg(new custom_interfaces::msg::FilteredSceneOctomap);
    octomap_filter_msg->header.frame_id = "world";
    octomap_filter_msg->header.stamp = now();
    int return_code = helpers::converters::pclToRosPtcld<pcl::PointXYZ>(scene_filtered, octomap_filter_msg->filtered_scene_octomap);
    return return_code == 0 ? std::move(octomap_filter_msg) : nullptr;
  }

  int OctomapFilter::_validateInput()
  {
    if (!_merged_ptcld_filtered_msg)
    {
      RCLCPP_WARN(get_logger(), "Module has not received data yet.");
      return 1;
    }
    if (_merged_ptcld_filtered_msg->header.stamp == builtin_interfaces::msg::Time())
    {
      RCLCPP_WARN(get_logger(), "Invalid input message header.");
      return 1;
    }
    return 0;
  }

  void OctomapFilter::_checkLastMessagesTimestamps(const builtin_interfaces::msg::Time &merged_items_stamp, const builtin_interfaces::msg::Time &merged_ptcld_filtered_stamp)
  {
    if (_last_merged_items_msg_timestamp == merged_items_stamp)
      RCLCPP_WARN(get_logger(), "New message with merged items has not arrived yet. Processing old message.");
    _last_merged_items_msg_timestamp = merged_items_stamp;

    if (_last_merged_ptcld_filtered_msg_timestamp == merged_ptcld_filtered_stamp)
      RCLCPP_WARN(get_logger(), "New message with filtered merged point cloud has not arrived yet. Processing old message.");
    _last_merged_ptcld_filtered_msg_timestamp = merged_ptcld_filtered_stamp;
  }

} // namespace octomap_filter
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_filter::OctomapFilter)
