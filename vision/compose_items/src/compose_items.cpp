#include "compose_items.hpp"

// #define test

namespace compose_items
{

    ComposeItems::ComposeItems(const rclcpp::NodeOptions &options, bool debug)
        : Node("compose_items", options)
    {
        (void)debug;
        status = custom_interfaces::msg::Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        helpers::commons::setLoggerLevelFromParameter(this);

        rcl_interfaces::msg::ParameterDescriptor parameter_descriptor;
        const std::string remove_shadows_param_name = "remove_shadows";
        parameter_descriptor.name = remove_shadows_param_name;
        parameter_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
        parameter_descriptor.read_only = false;
        this->declare_parameter(remove_shadows_param_name, false, parameter_descriptor);
        this->get_parameter<bool>(remove_shadows_param_name, _remove_shadows);

        RCLCPP_INFO(this->get_logger(), "started Node");
    }

    void ComposeItems::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;
        RCLCPP_INFO(this->get_logger(), "Initialization of compose items action server.");

        _getCamerasParameters();
        _readLabels();
        _readAreasParameters();
        this->_action_server = rclcpp_action::create_server<ComposeItemsAction>(
            this,
            "compose_items",
            std::bind(&ComposeItems::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ComposeItems::_handleCancel, this, std::placeholders::_1),
            std::bind(&ComposeItems::_handleAccepted, this, std::placeholders::_1));

        _detectron_client = this->create_client<custom_interfaces::srv::DataStoreDetectronSelect>("detectron_select");
        _rgbd_sync_client = this->create_client<custom_interfaces::srv::DataStoreRgbdSyncSelect>("rgbd_sync_select");
        _items_client = this->create_client<custom_interfaces::srv::DataStoreItemsInsert>("items_insert");

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

    void ComposeItems::_getData()
    {
        helpers::Timer timer(__func__, get_logger());

        auto detectron_request = std::make_shared<custom_interfaces::srv::DataStoreDetectronSelect::Request>();
        while (!_detectron_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            RCLCPP_INFO(this->get_logger(), "DataStore Detectron service not available, waiting again...");
        }

        auto detectron_result = _detectron_client->async_send_request(detectron_request);
        // Wait for the result.
        auto detectron_future_result = detectron_result.wait_for(5s);
        if (detectron_future_result == std::future_status::ready)
        {
            _detect_msg = std::make_shared<custom_interfaces::msg::DetectionsList>(detectron_result.get()->data);
            _detect_msg->header = detectron_result.get()->data.header;
        }
        else
            RCLCPP_ERROR(this->get_logger(), "Failed to read detectron data");

        auto rgbd_sync_request = std::make_shared<custom_interfaces::srv::DataStoreRgbdSyncSelect::Request>();
        while (!_rgbd_sync_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            RCLCPP_INFO(this->get_logger(), "DataStore RgbdSync service not available, waiting again...");
        }

        auto rgbd_sync_result = _rgbd_sync_client->async_send_request(rgbd_sync_request);
        // Wait for the result.
        auto rgbd_sync_future_result = rgbd_sync_result.wait_for(5s);
        if (rgbd_sync_future_result == std::future_status::ready)
        {
            _depth_images_msg = std::make_shared<custom_interfaces::msg::Images>();
            _rgb_images_msg = std::make_shared<custom_interfaces::msg::Images>();
            _depth_images_msg->data = rgbd_sync_result.get()->data.depths;
            _depth_images_msg->header.stamp = rgbd_sync_result.get()->data.header.stamp;
            _rgb_images_msg->data = rgbd_sync_result.get()->data.rgbs;
            _rgb_images_msg->header.stamp = rgbd_sync_result.get()->data.header.stamp;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read detectron data");
        }
    }

    void ComposeItems::_getCamerasParameters()
    {

        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");
        auto parameters = helpers::commons::getParameters({"cameras"});
        if (parameters.empty())
            RCLCPP_INFO(get_logger(), "cant read parameters from server...");
        else
        {
            _cameras_amount = parameters["cameras"]["cameras_amount"];
            RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        }

        auto get_camera_parameters = [this](std::string camera_frame)
        {
            std::optional<Eigen::Affine3f> camera_affine_opt;
            camera_affine_opt = helpers::vision::getCameraTransformAffine("world", camera_frame);
            if (camera_affine_opt == std::nullopt)
            {

                RCLCPP_WARN(this->get_logger(), "Compose items - cannot obtain transform to \"" + camera_frame + "\" shuting down node...");
                shutDownNode();
                return std::pair<Eigen::Affine3f, CameraParameters>();
            }
            Eigen::Affine3f cam_aff = *camera_affine_opt;
            CameraParameters camera_data;

            try
            {
                auto cam_intrinsic = helpers::vision::getCameraIntrinsic(get_node_topics_interface(), camera_frame);
                if (cam_intrinsic == std::nullopt)
                {

                    RCLCPP_WARN_STREAM(get_logger(), "Compose items - cannot obtain camera intrinsic to \"" + camera_frame + "\" shuting down node...");
                    shutDownNode();
                    return std::pair<Eigen::Affine3f, CameraParameters>();
                }
                camera_data.width = cam_intrinsic->width;
                camera_data.height = cam_intrinsic->height;
                camera_data.cx = cam_intrinsic->cx;
                camera_data.cy = cam_intrinsic->cy;
                camera_data.fx = cam_intrinsic->fx;
                camera_data.fy = cam_intrinsic->fy;
                RCLCPP_INFO(this->get_logger(), " Succesfully obtained camera parameters");
            }
            catch (json::exception &e)
            {
                RCLCPP_FATAL_STREAM(this->get_logger(), "Exception with ID: " << e.id << "; message: " << e.what());
                rclcpp::shutdown();
            }
            std::pair<Eigen::Affine3f, CameraParameters> camera_pair(cam_aff, camera_data);
            return camera_pair;
        };

        _cameras_data = std::make_shared<std::map<size_t, std::pair<Eigen::Affine3f, CameraParameters>>>();
        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
            (*_cameras_data)[cam_idx] = get_camera_parameters("camera_" + std::to_string(cam_idx));
    }

    void ComposeItems::_execute(const std::shared_ptr<GoalHandleComposeItems> goal_handle)
    {
        helpers::Timer timer(__func__, get_logger());
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        auto result = std::make_shared<ComposeItemsAction::Result>();

        _getData();
        if (_validateInputs(_detect_msg, _depth_images_msg, _rgb_images_msg))
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid input message. Goal failed.");
            goal_handle->abort(result);
            return;
        }

        _assignData(_detect_msg, _depth_images_msg, _rgb_images_msg);

        //main functionality
        composeItemsData();

        Response::SharedPtr compose_msg = std::make_shared<Response>();
        _save_data(compose_msg->data);
        _sendDataToDB(compose_msg);
        RCLCPP_INFO(this->get_logger(), "Executing succeed");
        goal_handle->succeed(result);
    }

    int ComposeItems::_sendDataToDB(Response::SharedPtr &compose_msg)
    {
        helpers::Timer timer(__func__, get_logger());
        auto request = std::make_shared<custom_interfaces::srv::DataStoreItemsInsert::Request>();
        while (!_items_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        request->data = compose_msg->data;
        auto data_store_result = _items_client->async_send_request(request);
        auto data_store_future_result = data_store_result.wait_for(5s);
        if (rclcpp::ok() && data_store_future_result == std::future_status::ready)
            return 0;
        return 1;
    }

    int ComposeItems::_readLabels()
    {
        helpers::Timer timer(__func__, get_logger());
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

    void ComposeItems::_computeElementData(std::string label, size_t el_idx, size_t cam_idx)
    {
        create_ptcld::createPtcld(_workspace_area, _items_data[cam_idx][label][el_idx], *(depths[cam_idx]), *(rgbs[cam_idx]), cam_idx, _cameras_data, _remove_shadows);
        _items_data[cam_idx][label][el_idx]._computeUBB(0.1);
    }

    int ComposeItems::_assignData(custom_interfaces::msg::DetectionsList::SharedPtr &detect_msg,
                                  custom_interfaces::msg::Images::SharedPtr &depth_images,
                                  custom_interfaces::msg::Images::SharedPtr &rgb_images)
    {
        helpers::Timer timer(__func__, get_logger());

#ifdef test
        /////////////////////////////////////test/////////////////
        std::cout << "Initial detections:" << std::endl;
        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
        {
            for (auto label : detect_msg->cameras[cam_idx - 1].labels)
            {
                std::cout << "camera: " << cam_idx << " label: " << label << std::endl;
            }
        }
        std::cout << "\n\n\n"
                  << std::endl;

/////////////////////////////////////test/////////////////
#endif

        std::vector<std::thread> workers(_cameras_amount * 2);
        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
        {
            cv::Size size;
            size.width = (*_cameras_data)[cam_idx].second.width;
            size.height = (*_cameras_data)[cam_idx].second.height;
            if (depths[cam_idx] == nullptr)
                depths[cam_idx] = std::make_shared<cv::Mat>(size, CV_32FC1);
            if (rgbs[cam_idx] == nullptr)
                rgbs[cam_idx] = std::make_shared<cv::Mat>(size, CV_8UC4);
            size_t idx = cam_idx - 1;
            workers[idx] = std::thread(std::bind(&helpers::converters::rosImageToCV, std::ref(rgb_images->data[idx]), std::ref(*rgbs[cam_idx])));
            workers[_cameras_amount + idx] = std::thread(std::bind(&helpers::converters::rosImageToCV, std::ref(depth_images->data[idx]), std::ref(*depths[cam_idx])));
        }

        _items_data.clear();
        _elements_data.clear();
        _matched_items.clear();
        _matched_elements.clear();

        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
        {
            size_t idx = cam_idx - 1;
            for (size_t i = 0; i < detect_msg->cameras[idx].labels.size(); i++)
            {
                std::string label = detect_msg->cameras[idx].labels[i];
                element el(detect_msg->cameras[idx].masks[i], label, cam_idx);
                if (_items[label])
                    _items_data[cam_idx][label].push_back(el);
                else
                    _elements_data[cam_idx][label].push_back(el);
            }
        }

        for (auto &worker : workers)
            worker.join();

        std::vector<std::thread> element_workers;
        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
            for (auto iter = _items_data[cam_idx].begin(); iter != _items_data[cam_idx].end(); iter++)
                for (size_t el_idx = 0; el_idx < _items_data[cam_idx][iter->first].size(); el_idx++)
                    element_workers.push_back(std::thread(std::bind(&ComposeItems::_computeElementData, this, iter->first, el_idx, cam_idx)));

        for (size_t i = 0; i < element_workers.size(); i++)
            element_workers[i].join();

        return 0;
    }

    bool ComposeItems::_initial_match(element &cam1_el, element &el)
    {
        float el1_width = cam1_el.max.x - cam1_el.min.x;
        float el1_height = cam1_el.max.y - cam1_el.min.y;
        float el2_width = el.max.x - el.min.x;
        float el2_height = el.max.y - el.min.y;

        return (cam1_el.min.x < el.min.x + el2_width &&
                cam1_el.min.x + el1_width > el.min.x &&
                cam1_el.min.y < el.min.y + el2_height &&
                cam1_el.min.y + el1_height > el.min.y);
    }

    void ComposeItems::_assignParent(element &parent, element &child)
    {
        if (!parent.matched)
            child.parent = &parent;
        else
            child.parent = parent.parent;

        child.matched = true;
        _matched_items[child.parent].push_back(&child);
    }

    void ComposeItems::_findAllChilds(label label, size_t parent_cam_idx, size_t child_cam_idx)
    {

        for (size_t parent_det_idx = 0; parent_det_idx < _items_data[parent_cam_idx][label].size(); parent_det_idx++)
            for (size_t child_det_idx = 0; child_det_idx < _items_data[child_cam_idx][label].size(); child_det_idx++)
            {
                element *parent = &(_items_data[parent_cam_idx][label][parent_det_idx]);

                element *child = &(_items_data[child_cam_idx][label][child_det_idx]);

                if (child->matched)
                    continue;

                if (!_initial_match(*parent, *child))
                    continue;

                create_ptcld::computeMaskForIndex(_cameras_data, parent_cam_idx, *child);
                if (!create_ptcld::compareMasks(parent->_masks[parent_cam_idx], child->_masks[parent_cam_idx], 0.1))
                    continue;

                _assignParent(*parent, *child);

                break;
            }
    }

    bool ComposeItems::_unpackMap(size_t &parent_cam_idx, size_t &child_cam_idx, std::string &label)
    {

        if(label.size() == 0)
            label = _items_data[parent_cam_idx].begin()->first;
        else {
            auto it = ++(_items_data[parent_cam_idx].find(label));
            if(it != _items_data[parent_cam_idx].end())
                label = it->first;
            else
                label.clear();
        }

        if (child_cam_idx >= _cameras_amount && label.size() == 0)
            child_cam_idx = ++parent_cam_idx;
        
        if (parent_cam_idx >= _cameras_amount)
            return false;

        if (child_cam_idx < _cameras_amount && label.size() == 0)
            child_cam_idx++;
        
        if(label.size() == 0)
            label = _items_data[parent_cam_idx].begin()->first;

        return true;
    }

    void ComposeItems::_matchDetectionsForEachCamera()
    {

        size_t parent_cam_idx = 1;
        size_t child_cam_idx = 1;
        std::string label = "";

        while (_unpackMap(parent_cam_idx, child_cam_idx, label))
            _findAllChilds(label, parent_cam_idx, child_cam_idx);
    }

    void ComposeItems::_matchElementsWithItem(element &parent)
    {

        for (auto element_label : _components[parent.label])
            for (size_t element_id = 0; element_id < _elements_data[parent.cam_idx][element_label].size(); element_id++)
            {
                element *child = &(_elements_data[parent.cam_idx][element_label][element_id]);

                if (child->matched || !create_ptcld::compareMasks(parent._masks[parent.cam_idx], child->_masks[parent.cam_idx], 0.2))
                    continue;

                child->matched = true;

                if (parent.parent)
                    child->parent = parent.parent;
                else
                    child->parent = &parent;

                _matched_elements[child->parent].push_back(child);

                break;
            }
    }

    void ComposeItems::_assignElements()
    {

        //unpack map - for each camera and each label
        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
            for (auto &item_label : _items_data[cam_idx])
                for (size_t item_id = 0; item_id < _items_data[cam_idx][item_label.first].size(); item_id++)
                    _matchElementsWithItem((_items_data[cam_idx][item_label.first][item_id]));
    }

    void ComposeItems::_passAllUnmatchedDetections()
    {

        for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
            for (auto &label : _items_data[cam_idx])
                for (size_t item_idx = 0; item_idx < _items_data[cam_idx][label.first].size(); item_idx++)
                {
                    element *item = &(_items_data[cam_idx][label.first][item_idx]);

                    if (!item->matched && _matched_items.find(item) == _matched_items.end())
                        _matched_items[item] = std::vector<element *>();
                }
        
    }

    void ComposeItems::composeItemsData()
    {
        helpers::Timer timer(__func__, get_logger());

        if (_cameras_amount > 1)
            _matchDetectionsForEachCamera();

        _passAllUnmatchedDetections();

        _assignElements();


    

#ifdef test

        std::cout << "\nitems: \n"
                  << std::endl;

        for (auto item : _matched_items)
        {
            std::cout << "parent cam id: " << item.first->cam_idx << " label: " << item.first->label << std::endl;
            for (auto child : item.second)
            {
                std::cout << "     child cam id: " << child->cam_idx << " label: " << child->label << std::endl;
            }
        }

        for (auto item : _matched_elements)
        {
            std::cout << "parent cam id: " << item.first->cam_idx << " label: " << item.first->label << std::endl;
            for (auto child : item.second)
            {
                std::cout << "     child cam id: " << child->cam_idx << " label: " << child->label << std::endl;
            }
        }
        std::cout << "\n\n\n"
                  << std::endl;

        std::cout << "found: " << _matched_items.size() << " unique items" << std::endl;
        std::cout << "found: " << _matched_elements.size() << " items with matched elements" << std::endl;

        size_t unmatched = 0;
        for (auto cam : _items_data)
            for (auto label : cam.second)
                for (auto det : label.second)
                    if (!det.matched && det.cam_idx != 1)
                        unmatched++;

        for (auto mat : _matched_items)
            if (mat.second.size() == 0)
                unmatched++;

        std::cout << "with: " << unmatched << " unmatched detections" << std::endl;

#endif
    }

    int ComposeItems::_logWarn(std::string msg)
    {
        RCLCPP_WARN(this->get_logger(), msg);
        return 1;
    }

    int ComposeItems::_validateInputs(custom_interfaces::msg::DetectionsList::SharedPtr &detect_msg, custom_interfaces::msg::Images::SharedPtr &depth_images_msg, custom_interfaces::msg::Images::SharedPtr &rgb_images_msg)
    {
        helpers::Timer timer(__func__, get_logger());
        if (!detect_msg || detect_msg->cameras.size() != _cameras_amount)
            return _logWarn("there is no valid data from detectron");

        if (!depth_images_msg || depth_images_msg->data.size() != _cameras_amount)
            return _logWarn("there is no valid depth data");

        if (!rgb_images_msg || rgb_images_msg->data.size() != _cameras_amount)
            return _logWarn("there is no valid rgb data");

        return 0;
    }

    void ComposeItems::_save_data(custom_interfaces::msg::Items &items)
    {   
        //TODO orange single camera bug

        helpers::Timer timer(__func__, get_logger());
        for (auto item : _matched_items)
        {

            custom_interfaces::msg::Item item_msg;
            item_msg.label = item.first->label;
            if (_elements.find(item.first->label) == _elements.end() || !_elements[item_msg.label])
            {
                for (auto el_label : _components[item.first->label])
                {

                    bool el_found = false;
                    custom_interfaces::msg::ItemElement element_msg;
                    element_msg.label = el_label;
                    element_msg.ptclds.resize(_cameras_amount);
                    element_msg.masks.resize(_cameras_amount);
                    for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
                    {
                        size_t idx = cam_idx - 1;
                        auto match = std::find_if(_matched_elements[item.first].begin(), _matched_elements[item.first].end(), [this, el_label, cam_idx](element *element)
                                                  { return element->label == el_label && element->cam_idx == cam_idx; });

                        if (match == _matched_elements[item.first].end())
                            continue;

                        create_ptcld::createPtcld(_workspace_area, **match, *(depths[cam_idx]), *(rgbs[cam_idx]), cam_idx, _cameras_data, _remove_shadows);

                        helpers::converters::pclToRosPtcld<pcl::PointXYZRGB>((*match)->_clouds[cam_idx], element_msg.ptclds[idx]);
                        element_msg.masks[idx] = (*match)->rle_mask;

                        _matched_elements[item.first].erase(match);
                        el_found = true;
                    }
                    if (el_found)
                        item_msg.item_elements.push_back(element_msg);
                }
            }
            else
            {
                std::set<std::string> el_labels;
                el_labels.insert(item.first->label);
                for (auto el : item.second)
                    el_labels.insert(el->label);

                for (auto el_label : el_labels)
                {
                    bool el_found = false;
                    custom_interfaces::msg::ItemElement element_msg;
                    element_msg.label = el_label;
                    element_msg.ptclds.resize(_cameras_amount);
                    element_msg.masks.resize(_cameras_amount);
                    for (size_t cam_idx = 1; cam_idx <= _cameras_amount; cam_idx++)
                    {

                        size_t idx = cam_idx - 1;
                        auto el_it = std::find_if(item.second.begin(), item.second.end(), [this, el_label, cam_idx](element *element)
                                                  { return element->label == el_label && element->cam_idx == cam_idx; });

                        element *found = nullptr;

                        if (el_it == item.second.end())
                        {
                            if (item.first->label == el_label && item.first->cam_idx == cam_idx)
                                found = item.first;
                            else
                                continue;
                        }
                        else
                            found = *el_it;

                        create_ptcld::createPtcld(_workspace_area, *found, *(depths[cam_idx]), *(rgbs[cam_idx]), cam_idx, _cameras_data, _remove_shadows);
                        helpers::converters::pclToRosPtcld<pcl::PointXYZRGB>(found->_clouds[cam_idx], element_msg.ptclds[idx]);

                        element_msg.masks[idx] = found->rle_mask;
                        if (el_it != item.second.end())
                            item.second.erase(el_it);
                        el_found = true;
                    }
                    if (el_found)
                        item_msg.item_elements.push_back(element_msg);
                }
            }
            items.items.push_back(item_msg);
        }
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(compose_items::ComposeItems)
