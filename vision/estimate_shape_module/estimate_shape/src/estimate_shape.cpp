#include "estimate_shape/estimate_shape.hpp"

namespace estimate_shape
{
    EstimateShape::EstimateShape(const rclcpp::NodeOptions &options)
        : Node("estimate_shape", options)
    {
        helpers::commons::setLoggerLevelFromParameter(this);
        RCLCPP_DEBUG_STREAM(LOGGER, "Using intra process communication: " << std::boolalpha << options.use_intra_process_comms() << std::noboolalpha);



        status = custom_interfaces::msg::Heartbeat::STOPPED;
        _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
        RCLCPP_INFO(LOGGER, "...estimate shape initialization done");
    }

    void EstimateShape::_getData()
    {

        auto items_request = std::make_shared<custom_interfaces::srv::DataStoreItemsSelect::Request>();
        while (!_items_select_client->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            }
            RCLCPP_INFO(this->get_logger(), "DataStore compose items service not available, waiting again...");
        }

        auto items_result = _items_select_client->async_send_request(items_request);
        // Wait for the result.
        if (items_result.wait_for(5s) == std::future_status::ready)
        {

            _compose_items_msg = std::make_shared<custom_interfaces::msg::Items>(items_result.get()->data);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to read compose items data");
        }
    }

    void EstimateShape::initNode()
    {
        status = custom_interfaces::msg::Heartbeat::STARTING;

        RCLCPP_INFO(LOGGER, "Estimate shape initialization...");
        this->_estimate_shape_server = rclcpp_action::create_server<SimpleAction>(
            this,
            "estimate_shape",
            std::bind(&EstimateShape::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&EstimateShape::_handleCancel, this, std::placeholders::_1),
            std::bind(&EstimateShape::_handleAccepted, this, std::placeholders::_1));

        auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)); //.transient_local().reliable();
        _estimate_shape_pub = create_publisher<Response>("estimate_shape", qos_settings);

        // _compose_items_sub = create_subscription<custom_interfaces::msg::Items>("compose_items", qos_settings,
        //                                                                         [this](custom_interfaces::msg::Items::SharedPtr compose_items_msg)
        //                                                                         {
        //                                                                             RCLCPP_DEBUG(get_logger(), "Composed items message received");
        //                                                                             _compose_items_msg = compose_items_msg;
        //                                                                         });
  
        _getParametersFromServer(_labels, _cams_params);
        if (status == custom_interfaces::msg::Heartbeat::STOPPED)
            return;
        
        _estimate_shape_manager = std::make_unique<EstimateShapeManager>(_cams_params, _labels);

        _items_select_client = this->create_client<custom_interfaces::srv::DataStoreItemsSelect>("items_select");
        _items_insert_client = this->create_client<custom_interfaces::srv::DataStoreItemsInsert>("items_insert");

        status = custom_interfaces::msg::Heartbeat::RUNNING;
    }

    void EstimateShape::shutDownNode()
    {
        status = custom_interfaces::msg::Heartbeat::STOPPING;

        status = custom_interfaces::msg::Heartbeat::STOPPED;
    }

    EstimateShape::~EstimateShape() {}

    void EstimateShape::_execute(const std::shared_ptr<GoalHandleSimpleAction> goal_handle)
    {
        helpers::Timer timer("Estimate shape action", get_logger());
        auto result = std::make_shared<SimpleAction::Result>();

        _getData();
        if (status != custom_interfaces::msg::Heartbeat::RUNNING)
        {
            RCLCPP_WARN_ONCE(this->get_logger(), "Node is not in running state");
            goal_handle->abort(result);
            return;
        }

        RCLCPP_INFO(LOGGER, "Executing goal");
        if (_validateInput(_compose_items_msg))
        {
            RCLCPP_ERROR(LOGGER, "Invalid input message. Goal failed.");
            // _estimate_shape_pub->publish(Response());
            goal_handle->abort(result);
            return;
        }

        if (_last_processed_msg_timestamp == _compose_items_msg->header.stamp)
            RCLCPP_WARN(get_logger(), "New message has not arrived yet. Processing old message.");
        _last_processed_msg_timestamp = _compose_items_msg->header.stamp;

        std::string item_label = "";
        std::string fit_method = "";
        try
        {
            auto estimate_shape_msg = _estimateShapeProcessing(_compose_items_msg, item_label, fit_method);
            // _estimate_shape_pub->publish(*estimate_shape_msg);
            auto request = std::make_shared<custom_interfaces::srv::DataStoreItemsInsert::Request>();
            request->data = estimate_shape_msg->data;

            // _items_insert_client
            auto data_store_result = _items_insert_client->async_send_request(request);
            auto data_store_future_result = data_store_result.wait_for(5s);
            if (rclcpp::ok() && data_store_future_result == std::future_status::ready)
            {
                goal_handle->succeed(result);
                RCLCPP_INFO(LOGGER, "Goal succeeded");
            }
            else
            {
                return;
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_STREAM(LOGGER, "Error occured during estimate shape: " << e.what());
            // _estimate_shape_pub->publish(Response());
            if (rclcpp::ok())
            {
                goal_handle->abort(result);
                RCLCPP_INFO(LOGGER, "Goal failed");
            }
        }
    }

    Response::SharedPtr EstimateShape::_estimateShapeProcessing(const ItemsMsg::SharedPtr &input_items, std::string item_label, std::string fit_method)
    {
        RCLCPP_INFO(LOGGER, "Estimate shape called");

        // Input processing
        std::vector<Item> items;
        RCLCPP_INFO(LOGGER, "Converting topic message to structures");
        _convertTopicDataToStructures(input_items, items);

        // Main logic
        RCLCPP_INFO(LOGGER, "Calling main estimate shape logic");
        _estimate_shape_manager->estimateShape(items, item_label, fit_method);

        // Output processing
        Response::SharedPtr estimate_shape_msg = _prepareOutputData(input_items);
        RCLCPP_INFO(LOGGER, "Converting structures to topic message");
        _convertStructuresToTopicData(items, estimate_shape_msg);

        return estimate_shape_msg;
    }

    int EstimateShape::_convertTopicDataToStructures(const ItemsMsg::SharedPtr &input_items, std::vector<Item> &out_items)
    {
        out_items.resize(input_items->items.size());
        for (size_t item_idx = 0; item_idx < input_items->items.size(); ++item_idx)
        {
            const auto &topic_item = input_items->items[item_idx];

            Item &item_structure = out_items[item_idx];
            item_structure.id = topic_item.id;
            item_structure.label = topic_item.label;

            item_structure.item_elements.resize(topic_item.item_elements.size());
            for (size_t item_element_idx = 0; item_element_idx < topic_item.item_elements.size(); ++item_element_idx)
            {
                const auto &topic_item_element = topic_item.item_elements[item_element_idx];

                ItemElement &item_element_structure = item_structure.item_elements[item_element_idx];
                item_element_structure.id = topic_item_element.id;
                item_element_structure.item_id = topic_item.id;
                item_element_structure.element_label = topic_item_element.label;

              
                item_element_structure.pclds_rgb.resize(topic_item_element.ptclds.size());
                for(size_t i=0; i< item_element_structure.pclds_rgb.size(); i++)
                    helpers::converters::rosPtcldtoPcl<pcl::PointXYZRGB>(topic_item_element.ptclds[i], item_element_structure.pclds_rgb[i]);
                
                for(auto &rgb_cloud:item_element_structure.pclds_rgb)
                    pcl::copyPointCloud(*rgb_cloud, *item_element_structure.pcl_merged);

                // helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(topic_item_element.ptcld, item_element_structure.pcl_merged);
                // helpers::converters::rosPtcldtoPcl<pcl::PointXYZRGB>(topic_item_element.cam2_ptcld, item_element_structure.element_pcl_2);
                // helpers::converters::rosPtcldtoPcl<pcl::PointXYZ>(topic_item_element.merged_ptcld, item_element_structure.pcl_merged);
            }
        }
        return 0;
    }

    int EstimateShape::_convertStructuresToTopicData(const std::vector<Item> &item_results, Response::SharedPtr &out_estimate_shape)
    {

        for (auto topic_item_it = out_estimate_shape->data.items.begin(); topic_item_it != out_estimate_shape->data.items.end();)
        {
            auto item_result_iter = std::find_if(item_results.begin(), item_results.end(), [topic_item_it](const Item &item_result)
                                                 { return item_result.id == topic_item_it->id; });
            if (!item_result_iter->isEstimationValid)
            {
                LOG_DEBUG_STREAM("Item with ID " << item_result_iter->id << " has not valid estimation and will not be published.");
                topic_item_it = out_estimate_shape->data.items.erase(topic_item_it);
                continue;
            }

            // Valid estimation
            helpers::converters::eigenAffineToGeometry(item_result_iter->pose, topic_item_it->pose);
            for (auto &topic_item_element : topic_item_it->item_elements)
            {
                auto item_element_result_iter = std::find_if(item_result_iter->item_elements.begin(), item_result_iter->item_elements.end(),
                                                             [topic_item_element](const ItemElement &item_element_result)
                                                             { return item_element_result.id == topic_item_element.id; });
                if (item_element_result_iter != item_result_iter->item_elements.end())
                {
                    topic_item_element.parts_description.clear();
                    for (auto &part_description : item_element_result_iter->parts_description)
                    {
                        if (!part_description.empty())
                            topic_item_element.parts_description.push_back(part_description.dump());
                    }
                }
            }
            ++topic_item_it;
        }

        return 0;
    }

    Response::UniquePtr EstimateShape::_prepareOutputData(const ItemsMsg::SharedPtr &input_items)
    {
        Response::UniquePtr estimate_shape_data = std::make_unique<Response>();
        estimate_shape_data->data.items = input_items->items;
        estimate_shape_data->data.header.frame_id = "world";
        estimate_shape_data->data.header.stamp = now();
        return estimate_shape_data;
    }

    int EstimateShape::_getParametersFromServer(std::vector<Label> &out_labels_parameters, std::vector<estimate_shape::CameraParameters> &out_cameras_parameters)
    {
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
                label_parameters.raw_data = label_params;
                out_labels_parameters.push_back(label_parameters);
            }
        }
        catch (const json::exception &e)
        {
            throw std::runtime_error("Error occured while parsing \"labels\" from parameter server");
        }

        // Camera affines
        auto get_camera_affine = [this](const std::string &camera_frame) -> std::optional<CameraParameters>
        {
            // std::optional<Eigen::Affine3f> camera_affine_opt;
            //     camera_affine_opt = helpers::vision::getCameraTransformAffine("world");
            //     // if (camera_affine_opt)
            //     //     break;
            //     // RCLCPP_WARN_STREAM_THROTTLE(LOGGER, *get_clock(), 1000, "Estimate shape - cannot obtain transform to \"" + camera_frame + "\" trying again...");
            
            // if (camera_affine_opt == std::nullopt)
            // {
            //     camera_affine_opt = helpers::vision::getCameraTransformAffine("world");
            //     if (camera_affine_opt == std::nullopt)
            //     {

            //         RCLCPP_WARN(this->get_logger(), "Estimate shape - cannot obtain transform to \"" + camera_frame + "\" shuting down node...");
            //         return std::nullopt;
            //     }
            // }

            std::optional<Eigen::Affine3f> camera_affine_opt;
            camera_affine_opt = helpers::vision::getCameraTransformAffine("world", camera_frame);
            if (camera_affine_opt == std::nullopt)
            {

                RCLCPP_WARN(this->get_logger(), "Compose items - cannot obtain transform to \"" + camera_frame + "\" shuting down node...");
                shutDownNode();
                return std::nullopt;
            }

                    
            Eigen::Translation3f translation(camera_affine_opt->translation());
            Eigen::Quaternionf quaternion(camera_affine_opt->rotation());
            return std::optional<CameraParameters>(std::in_place, translation, quaternion, camera_frame);
        };

        size_t cameras_amount = 0;
        RCLCPP_INFO_ONCE(get_logger(), "Reading parameters from the server");
        auto parameters = helpers::commons::getParameters({"cameras"});
        if (parameters.empty())
            RCLCPP_INFO(get_logger(), "cant read parameters from server...");
        else
        {
            cameras_amount = parameters["cameras"]["cameras_amount"];
            RCLCPP_INFO(get_logger(), "Parameters read successfully...");
        }


        for(size_t cam_idx = 1; cam_idx <= cameras_amount; cam_idx++){

        if (auto cam_params = get_camera_affine("camera_" + std::to_string(cam_idx)))
            out_cameras_parameters.push_back(*cam_params);
        else
            shutDownNode();
        }
      

        return 0;
    }

    rclcpp_action::GoalResponse EstimateShape::_handleGoal(const rclcpp_action::GoalUUID & /*uuid*/, std::shared_ptr<const SimpleAction::Goal> /*goal*/)
    {
        RCLCPP_INFO(LOGGER, "Received goal request with order ");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse EstimateShape::_handleCancel(const std::shared_ptr<GoalHandleSimpleAction> /*goal_handle*/)
    {
        RCLCPP_INFO(LOGGER, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void EstimateShape::_handleAccepted(const std::shared_ptr<GoalHandleSimpleAction> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&EstimateShape::_execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    int EstimateShape::_validateInput(const ItemsMsg::SharedPtr &input_items)
    {
        if (!input_items)
        {
            RCLCPP_WARN(LOGGER, "Module has not received data yet.");
            return 1;
        }
        // if (input_items->header.stamp == builtin_interfaces::msg::Time())
        // {
        //     RCLCPP_WARN(LOGGER, "Invalid input message header.");
        //     return 1;
        // }
        return 0;
    }
} // namespace estimate_shape

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(estimate_shape::EstimateShape)
