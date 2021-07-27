#include "compose_items_server.hpp"

namespace compose_items
{

    ComposeItems::ComposeItems(const rclcpp::NodeOptions &options, bool debug)
        : Node("compose_items_server", options),
          _scene(new current_scene_t),
          _leaf_size(0.005),
          _create_ptcld(new robot::CreatePtcld),
          _debug(debug)
    {
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        RCLCPP_INFO(this->get_logger(), "started Node");
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    }


    void ComposeItems::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        helpers::commons::setLoggerLevelFromParameter(this);
        _getCamerasParameters();
        RCLCPP_INFO(this->get_logger(), "Initialization of compose items action server.");
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)); //.transient_local().reliable();
        _publisher = this->create_publisher<custom_interfaces::msg::Items>("compose_items", qos_settings);
        _initializeSubscribers(qos_settings);
        _readLabels();
        _readAreasParameters();
        this->_action_server = rclcpp_action::create_server<ComposeItemsAction>(
            this,
            "compose_items",
            std::bind(&ComposeItems::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ComposeItems::_handleCancel, this, std::placeholders::_1),
            std::bind(&ComposeItems::_handleAccepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Compose items action server Inicialized.");
        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void ComposeItems::shutDownNode()
    {
        RCLCPP_INFO(this->get_logger(), "shut Down Node");
        if (status != custom_interfaces::msg::Heartbeat::STOPPED)
            status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    ComposeItems::~ComposeItems()
    {
        shutDownNode();
    }


    void ComposeItems::_initializeSubscribers(const rclcpp::QoS &qos_settings)
    {
        _new_masks_subscriber = create_subscription<custom_interfaces::msg::Detections>("new_masks", qos_settings,
                                                                                        [this](custom_interfaces::msg::Detections::SharedPtr detections_msg)
                                                                                        {
                                                                                            RCLCPP_DEBUG(get_logger(), "New masks message received");
                                                                                            _detect_msg = detections_msg;
                                                                                        });
        _rgb_images_subscriber = create_subscription<custom_interfaces::msg::RgbImages>("rgb_images", qos_settings,
                                                                                        [this](custom_interfaces::msg::RgbImages::SharedPtr rgb_images_msg)
                                                                                        {
                                                                                            RCLCPP_DEBUG(get_logger(), "RGB images received message received");
                                                                                            _rgb_images_msg = rgb_images_msg;
                                                                                        });
        _depth_images_subscriber = create_subscription<custom_interfaces::msg::DepthImages>("depth_images", qos_settings,
                                                                                            [this](custom_interfaces::msg::DepthImages::SharedPtr depth_images_msg)
                                                                                            {
                                                                                                RCLCPP_DEBUG(get_logger(), "Depth images received message received");
                                                                                                _depth_images_msg = depth_images_msg;
                                                                                            });
    }

    rclcpp_action::GoalResponse ComposeItems::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const ComposeItemsAction::Goal> /*goal*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ComposeItems::_handleCancel(const std::shared_ptr<GoalHandleComposeItems> /*goal_handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ComposeItems::_handleAccepted(const std::shared_ptr<GoalHandleComposeItems> goal_handle)
    {
        std::thread{std::bind(&ComposeItems::_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void ComposeItems::_getCamerasParameters()
    {
        auto get_camera_parameters = [this](std::string camera_frame)
        {
            std::optional<Eigen::Affine3f> camera_affine_opt;
            while (true)
            {
                camera_affine_opt = helpers::vision::getCameraTransformAffine("world", camera_frame);
                if (camera_affine_opt)
                    break;
                RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Compose items - cannot obtain transform to \"" + camera_frame + "\" trying again...");
            }
            Eigen::Affine3f cam_aff = *camera_affine_opt;

            CameraParameters camera_data;
            // try
            // {
            //     auto cam_intrinsic = helpers::commons::getCameraIntrinsic(get_node_topics_interface(), camera_frame);
            //     if (!cam_intrinsic)
            //     {
            //         rclcpp::shutdown();
            //         throw std::runtime_error("Compose items - cannot obtain intrinsic parameters from " + camera_frame + " camera");
            //     }
            //     camera_data.width = cam_intrinsic->width;
            //     camera_data.height = cam_intrinsic->height;
            //     camera_data.cx = cam_intrinsic->cx;
            //     camera_data.cy = cam_intrinsic->cy;
            //     camera_data.fx = cam_intrinsic->fx;
            //     camera_data.fy = cam_intrinsic->fy;
            // }
            // catch (json::exception &e)
            // {
            //     RCLCPP_FATAL_STREAM(this->get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
            //     rclcpp::shutdown();
            // }

            for (;;)
            {
                try
                {
                    auto cam_intrinsic = helpers::vision::getCameraIntrinsic(get_node_topics_interface(), camera_frame);
                    if (!cam_intrinsic)
                    {
                        RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "Compose items - cannot obtain camera intrinsic to \"" + camera_frame + "\" trying again...");
                        continue;
                        // rclcpp::shutdown();
                        // throw std::runtime_error("Compose items - cannot obtain intrinsic parameters from " + camera_frame + " camera");
                    }
                    camera_data.width = cam_intrinsic->width;
                    camera_data.height = cam_intrinsic->height;
                    camera_data.cx = cam_intrinsic->cx;
                    camera_data.cy = cam_intrinsic->cy;
                    camera_data.fx = cam_intrinsic->fx;
                    camera_data.fy = cam_intrinsic->fy;
                    RCLCPP_INFO(this->get_logger(), " Succesfully obtained camera parameters");
                    break;
                }
                catch (json::exception &e)
                {
                    RCLCPP_FATAL_STREAM(this->get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
                    rclcpp::shutdown();
                }
            }

            std::pair<Eigen::Affine3f, CameraParameters> camera_pair(cam_aff, camera_data);
            return camera_pair;
        };

        auto cam1_data = get_camera_parameters("camera_1");
        auto cam2_data = get_camera_parameters("camera_2");
        transform_map camera_transform;
        Frames frames;
        camera_transform[frames.camera_frame] = cam1_data.first;
        camera_transform[frames.camera_frame2] = cam2_data.first;
        std::map<std::string, CameraParameters> camera_parameters;
        camera_parameters["camera_1"] = cam1_data.second;
        camera_parameters["camera_2"] = cam2_data.second;

        _create_ptcld->setCameraParams(camera_transform, camera_parameters);
    }

    int ComposeItems::_saveComposedData(custom_interfaces::msg::Items::UniquePtr &compose_msg)
    {
        compose_msg->items.resize(_items_to_save.size());

        std::vector<size_t> empty_items;
        for (size_t i = 0; i < _items_to_save.size(); i++)
        {
            size_t elements_clouds_size = 0;

            uint32_t id = _items_to_save[i].item_id;
            compose_msg->items[i].id = _items_to_save[i].item_id;
            compose_msg->items[i].label = _items_to_save[i].label;

            //TODO there should be no items without element - if there is no matching element - something went wrong and we should catch it here.
            std::vector<element_t>::iterator el_it = std::find_if(_elements_to_save.begin(), _elements_to_save.end(), [id](const element_t &el)
                                                                  { return el.item_id == id; });

            while (el_it != _elements_to_save.end())
            {
                custom_interfaces::msg::ItemElement element;
                element.id = el_it->item_element_id;
                element.label = el_it->element_label;
                helpers::converters::pclToRosPtcld<pcl::PointXYZRGB>(el_it->element_pcl_1, element.cam1_ptcld);
                helpers::converters::pclToRosPtcld<pcl::PointXYZRGB>(el_it->element_pcl_2, element.cam2_ptcld);
                helpers::converters::pclToRosPtcld<pcl::PointXYZ>(el_it->pcl_merged, element.merged_ptcld);
                helpers::converters::pclToRosPtcld<pcl::PointXYZRGB>(el_it->shadow_ptcld, element.shadow_ptcld);
                elements_clouds_size += el_it->pcl_merged->points.size();
                std::shared_ptr<std::iostream> mask1_stream;

                if (el_it->element_mask_1)
                {
                    helpers::converters::binaryMaskToStream(el_it->element_mask_1, mask1_stream);
                    helpers::converters::iostreamToString(mask1_stream, element.cam1_mask);
                }

                std::shared_ptr<std::iostream> mask2_stream;
                if (el_it->element_mask_2)
                {
                    helpers::converters::binaryMaskToStream(el_it->element_mask_2, mask2_stream);
                    helpers::converters::iostreamToString(mask2_stream, element.cam2_mask);
                }

                // if (el_it->element_depth_1)
                //     helpers::converters::cvMatToRos(*(el_it->element_depth_1), element.cam1_depth);

                // if (el_it->element_depth_2)
                //     helpers::converters::cvMatToRos(*(el_it->element_depth_2), element.cam2_depth);

                compose_msg->items[i].item_elements.push_back(element);
                _elements_to_save.erase(el_it);
                if (el_it == _elements_to_save.end())
                    break;
                el_it = std::find_if(_elements_to_save.begin(), _elements_to_save.end(), [id](const element_t &el)
                                     { return el.item_id == id; });
            }
            if (elements_clouds_size < 10)
                empty_items.push_back(i);
        }

        std::sort(empty_items.begin(), empty_items.end(), std::greater<size_t>());
        for (auto &idx : empty_items)
            compose_msg->items.erase(compose_msg->items.begin() + idx);

        compose_msg->header.stamp = now();
        compose_msg->header.frame_id = "world";

        return 0;
    }

    void ComposeItems::_execute(const std::shared_ptr<GoalHandleComposeItems> goal_handle)
    {
        helpers::Timer timer("Compose items action", get_logger());
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto result = std::make_shared<ComposeItemsAction::Result>();

        if (_validateInputs(_detect_msg, _depth_images_msg))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid input message. Goal failed.");
            _publisher->publish(custom_interfaces::msg::Items());
            goal_handle->abort(result);
            return;
        }

        _checkLastMessagesTimestamps();

        _assignData(_detect_msg, _depth_images_msg);

        //main functionality
        composeItemsData();
        //output data
        custom_interfaces::msg::Items::UniquePtr compose_msg(new custom_interfaces::msg::Items);
        _saveComposedData(compose_msg);

        _publisher->publish(std::move(compose_msg));

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

    int ComposeItems::_readLabels()
    {

        try
        {
            json camera_max_distance = helpers::commons::getParameter("compose_items");
            _workspace_area.camera_max_distance = camera_max_distance["camera_depth_distance"].get<float>();
        }
        catch (const json::exception &e)
        {
            RCLCPP_FATAL_STREAM(get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
            throw std::runtime_error("Compose item cant obtain camera depth distance from JSON error.");
        }
        json labels_params = helpers::commons::getParameter("labels");
        for (auto label : labels_params)
        {
            json label_data = label;
            std::string current_label;
            if (label_data.contains("label"))
            {
                current_label = label_data["label"].get<std::string>();
                _labels.push_back(current_label);
            }
            if (label_data.contains("item") && label_data.contains("element"))
            {
                bool item = label_data["item"].get<bool>();
                _items[current_label] = item;
                bool element = label_data["element"].get<bool>();
                _elements[current_label] = element;

                if (!element)
                {
                    std::vector<std::string> components = label_data["components"];
                    for (auto compo : components)
                        _components[current_label] = components;
                }
            }
        }
        return 0;
    }

    int ComposeItems::_assignData(custom_interfaces::msg::Detections::SharedPtr &detect_msg, custom_interfaces::msg::DepthImages::SharedPtr &depth_images_msg)
    {
        //TODO debug - timer and check sizes of detections
        _scene = std::make_shared<current_scene_t>();
        helpers::converters::rosImageToCV(_rgb_images_msg->cam1_rgb, _scene->camera_1_rgb);
        helpers::converters::rosImageToCV(_rgb_images_msg->cam2_rgb, _scene->camera_2_rgb);
        helpers::converters::rosImageToCV(depth_images_msg->cam1_depth, _scene->camera_1_depth);
        helpers::converters::rosImageToCV(depth_images_msg->cam2_depth, _scene->camera_2_depth);

        //camera 1
        _detections_cam1.clear();
        _detections_cam1.resize(detect_msg->cam1_masks.size());
        for (size_t i = 0; i < detect_msg->cam1_masks.size(); i++)
        {
            _detections_cam1[i].label = detect_msg->cam1_labels[i];
            std::shared_ptr<std::stringstream> mask_stream(new std::stringstream);
            *mask_stream << detect_msg->cam1_masks[i];
            helpers::converters::streamToBinaryMask(mask_stream, _detections_cam1[i].mask);
            cv::resize(_detections_cam1[i].mask, _detections_cam1[i].mask, _scene->camera_1_depth.size(), 0, 0, cv::INTER_NEAREST);
            _create_ptcld->cutDepthMapWithMask(_detections_cam1[i].mask, _scene->camera_1_depth, _detections_cam1[i].depth_data);
            _create_ptcld->createPtcld(_workspace_area, _detections_cam1[i].mask, _detections_cam1[i].depth_data, _scene->camera_1_rgb, 1, _detections_cam1[i].pcl_data, _detections_cam1[i].pcl_shadow);
        }

        //camera 2
        _detections_cam2.clear();
        _detections_cam2.resize(detect_msg->cam2_masks.size());
        for (size_t i = 0; i < detect_msg->cam2_masks.size(); i++)
        {
            _detections_cam2[i].label = detect_msg->cam2_labels[i];
            std::shared_ptr<std::stringstream> mask_stream(new std::stringstream);
            *mask_stream << detect_msg->cam2_masks[i];
            helpers::converters::streamToBinaryMask(mask_stream, _detections_cam2[i].mask);
            cv::resize(_detections_cam2[i].mask, _detections_cam2[i].mask, _scene->camera_2_depth.size(), 0, 0, cv::INTER_NEAREST);
            _create_ptcld->cutDepthMapWithMask(_detections_cam2[i].mask, _scene->camera_2_depth, _detections_cam2[i].depth_data);
            _create_ptcld->createPtcld(_workspace_area, _detections_cam2[i].mask, _detections_cam2[i].depth_data, _scene->camera_2_rgb, 2, _detections_cam2[i].pcl_data, _detections_cam2[i].pcl_shadow);
        }

        return 0;
    }

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> ComposeItems::_checkPtcld(std::shared_ptr<item_cam_t> item_cam_ptr)
    {
        if (!item_cam_ptr)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            return empty_cloud;
        }
        return item_cam_ptr->pcl_data;
    }

    std::shared_ptr<cv::Mat> ComposeItems::_getMaskPtr(std::shared_ptr<item_cam_t> item_cam_ptr)
    {
        if (!item_cam_ptr)
            return std::shared_ptr<cv::Mat>(nullptr);
        return std::make_shared<cv::Mat>(item_cam_ptr->mask);
    }

    std::shared_ptr<cv::Mat> ComposeItems::_getDepthPtr(std::shared_ptr<item_cam_t> item_cam_ptr)
    {
        if (!item_cam_ptr)
            return std::shared_ptr<cv::Mat>(nullptr);
        return std::make_shared<cv::Mat>(item_cam_ptr->depth_data);
    }

    bool ComposeItems::_checkOverlay(cv::Mat &item_mask, cv::Mat &element_mask)
    {
        if (item_mask.empty() || element_mask.empty())
            return false;
        cv::Mat res;
        int size = cv::countNonZero(item_mask);
        cv::bitwise_and(item_mask, element_mask, res);
        int size_res = cv::countNonZero(res);
        if (size_res > 0.2 * size)
            return true;
        return false;
    }

    bool ComposeItems::_isComponent(detected_item_t &item, element_t &element, const std::vector<element_t> &elements)
    {
        bool overlay = false;
        std::vector<std::string> components = _components[item.label];
        auto it = std::find(components.begin(), components.end(), element.element_label);
        if (it != components.end())
            components.erase(it);

        if (item.item_cam1 && element.element_mask_1)
        {
            cv::Mat item_mask = cv::Mat::zeros(element.element_mask_1->rows, element.element_mask_1->cols, CV_8UC1);
            for (auto &el : elements)
                if (std::find(components.begin(), components.end(), el.element_label) != components.end())
                    if (el.element_mask_1)
                        cv::bitwise_or(item_mask, *(el.element_mask_1), item_mask);

            cv::bitwise_and(item_mask, item.item_cam1->mask, item_mask);
            item_mask = 1 - item_mask;
            cv::bitwise_and(item_mask, item.item_cam1->mask, item_mask);

            overlay = _checkOverlay(item_mask, *(element.element_mask_1));
        }

        if (item.item_cam2 && element.element_mask_2)
        {
            cv::Mat item_mask = cv::Mat::zeros(element.element_mask_2->rows, element.element_mask_2->cols, CV_8UC1);
            for (auto &el : elements)
                if (std::find(components.begin(), components.end(), el.element_label) != components.end())
                    if (el.element_mask_2)
                        cv::bitwise_or(item_mask, *(el.element_mask_2), item_mask);

            cv::bitwise_and(item_mask, item.item_cam2->mask, item_mask);
            item_mask = 1 - item_mask;
            cv::bitwise_and(item_mask, item.item_cam2->mask, item_mask);

            overlay = (overlay || _checkOverlay(item_mask, *(element.element_mask_2)));
        }
        return overlay;
    }

    int ComposeItems::_assignElement(std::string label, detected_item_t &item, std::vector<element_t> &out_elements)
    {
        element_t current_element(_checkPtcld(item.item_cam1), _checkPtcld(item.item_cam2));
        current_element.item_element_id = ++_element_pk;
        current_element.item_id = _items[label] ? _starting_index : 0;
        current_element.element_label = label;
        current_element.element_mask_1 = _getMaskPtr(item.item_cam1);
        current_element.element_mask_2 = _getMaskPtr(item.item_cam2);
        current_element.element_depth_1 = _getDepthPtr(item.item_cam1);
        current_element.element_depth_2 = _getDepthPtr(item.item_cam2);
        if (item.item_cam1)
        {
            *current_element.shadow_ptcld = *item.item_cam1->pcl_shadow;
        }
        if (item.item_cam2)
        {
            *current_element.shadow_ptcld = *item.item_cam2->pcl_shadow;
        }

        out_elements.push_back(current_element);
        return 0;
    }

    int ComposeItems::_assignItem(uint32_t camera_index, item_cam_t &detection, detected_item_t &out_item)
    {
        if (_items[detection.label])
        {
            out_item.item_id = ++_starting_index;
            out_item.label = detection.label;
            out_item.item_id_hash = out_item.label + std::to_string(_starting_index);
        }
        out_item.item_cam1 = std::make_shared<item_cam_t>(detection);
        uint32_t matched_index;
        if (camera_index == 1 && !_create_ptcld->findCorespondingItem(detection, _detections_cam2, matched_index))
        {
            out_item.item_cam2 = std::make_shared<item_cam_t>(_detections_cam2[matched_index]);
            _matched_cam2_indices.push_back(matched_index);
        }
        else if (camera_index == 2)
        {
            out_item.item_cam1 = std::shared_ptr<item_cam_t>(nullptr);
            out_item.item_cam2 = std::make_shared<item_cam_t>(detection);
        }
        else
            out_item.item_cam2 = std::shared_ptr<item_cam_t>(nullptr);
        return 0;
    }

    void ComposeItems::composeItemsData()
    {
        if (_detections_cam1.size() + _detections_cam2.size() == 0)
            return;
        _items_to_save.clear();
        _remaining_items.clear();
        _elements_to_save.clear();

        {
            Debug debug("Main Loop started", _debug, "Exectuion time for main loop:");

            //camera 1
            for (auto &detection_cam1 : _detections_cam1)
            {
                detected_item_t current_item;
                _assignItem(1, detection_cam1, current_item);

                if (_items[detection_cam1.label])
                    _items_to_save.push_back(current_item);
                else
                    _remaining_items.push_back(current_item);

                if (_elements[detection_cam1.label])
                    _assignElement(detection_cam1.label, current_item, _elements_to_save);
            }
            //camera 2
            for (uint32_t i = 0; i < _detections_cam2.size(); i++)
            {
                if (std::find(std::begin(_matched_cam2_indices), std::end(_matched_cam2_indices), i) == _matched_cam2_indices.end())
                {
                    detected_item_t current_item;
                    _assignItem(2, _detections_cam2[i], current_item);

                    if (_items[_detections_cam2[i].label])
                        _items_to_save.push_back(current_item);
                    else
                        _remaining_items.push_back(current_item);

                    if (_elements[_detections_cam2[i].label])
                        _assignElement(_detections_cam2[i].label, current_item, _elements_to_save);
                }
            }
            //find correspondence
            for (auto &item : _items_to_save)
            {
                if (_components[item.label].size() > 0)
                {

                    for (auto &element_lable : _components[item.label])
                    {

                        for (auto &element : _elements_to_save)
                        {
                            if (element.element_label == element_lable && _isComponent(item, element, _elements_to_save))
                            {
                                element.item_id = item.item_id;
                            }
                        }
                    }
                }
            }
        }
    }

    int ComposeItems::_validateInputs(custom_interfaces::msg::Detections::SharedPtr &detect_msg, custom_interfaces::msg::DepthImages::SharedPtr &depth_images_msg)
    {
        auto checkHeader = [](const std_msgs::msg::Header &header)
        {
            return header.stamp == builtin_interfaces::msg::Time();
        };
        if (!detect_msg || checkHeader(detect_msg->header) || !depth_images_msg || checkHeader(depth_images_msg->header))
            return 1;
        return 0;
    }

    void ComposeItems::_checkLastMessagesTimestamps()
    {
        if (_last_rgb_images_processed_msg_timestamp == _rgb_images_msg->header.stamp)
            RCLCPP_WARN(get_logger(), "New message with RGB images has not arrived yet. Processing old message.");
        _last_rgb_images_processed_msg_timestamp = _rgb_images_msg->header.stamp;

        if (_last_depth_images_processed_msg_timestamp == _depth_images_msg->header.stamp)
            RCLCPP_WARN(get_logger(), "New message with Depth images has not arrived yet. Processing old message.");
        _last_depth_images_processed_msg_timestamp = _depth_images_msg->header.stamp;

        if (_last_detections_processed_msg_timestamp == _detect_msg->header.stamp)
            RCLCPP_WARN(get_logger(), "New message with detections has not arrived yet. Processing old message.");
        _last_detections_processed_msg_timestamp = _detect_msg->header.stamp;
    }

    void ComposeItems::_readAreasParameters()
    {
        try
        {
            json area = helpers::commons::getParameter("areas");
            json data_type = helpers::commons::getParameter("detect");

            _workspace_area.x_min = area["table_area"]["min"]["x"].get<float>();
            _workspace_area.y_min = area["table_area"]["min"]["y"].get<float>();
            _workspace_area.z_min = area["table_area"]["min"]["z"].get<float>();

            _workspace_area.x_max = area["table_area"]["max"]["x"].get<float>();
            _workspace_area.y_max = area["table_area"]["max"]["y"].get<float>();
            _workspace_area.z_max = area["table_area"]["max"]["z"].get<float>();
            _workspace_area.data_type = data_type["model"].get<std::string>();
        }
        catch (const json::exception &e)
        {
            RCLCPP_FATAL_STREAM(get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
            throw std::runtime_error("Get occupancy grid reading grid size from JSON error.");
        }
    }
}
// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto action_server = std::make_shared<ComposeItems>();
//     rclcpp::spin(action_server);
//     rclcpp::shutdown();
//     return 0;
// }

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(compose_items::ComposeItems)
