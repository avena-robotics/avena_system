#include "filter_detections.hpp"

FilterDetection::FilterDetection(const rclcpp::NodeOptions &options, bool debug)
    : Node("filter_detections_node", options),
      _debug(debug)
{
    helpers::commons::setLoggerLevelFromParameter(this);

    RCLCPP_INFO(this->get_logger(), "Initialization of filterdetections.");
    rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)); //.transient_local().reliable();
    _detections_sub = create_subscription<custom_interfaces::msg::Detections>("detections", qos_settings,
                                                                              [this](const custom_interfaces::msg::Detections::SharedPtr detections)
                                                                              {
                                                                                  RCLCPP_DEBUG(this->get_logger(), "Detections message received");
                                                                                  _detections_msg = detections;
                                                                              });
    _publisher = this->create_publisher<custom_interfaces::msg::Detections>("detections_filtered", qos_settings);
    _detectron_insert = this->create_client<custom_interfaces::srv::DataStoreDetectronInsert>("filter_detections");
    _readLabels();

    this->_action_server = rclcpp_action::create_server<FilterDetectionAction>(
        this,
        "filter_detections",
        std::bind(&FilterDetection::_handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&FilterDetection::_handleCancel, this, std::placeholders::_1),
        std::bind(&FilterDetection::_handleAccepted, this, std::placeholders::_1));

    _watchdog = std::make_shared<helpers::Watchdog>(this, this, "system_monitor");
    status = custom_interfaces::msg::Heartbeat::STOPPED;

    RCLCPP_INFO(this->get_logger(), "Filter detections action server Inicialized.");
}

void FilterDetection::initNode()
{
    status = custom_interfaces::msg::Heartbeat::STARTING;

    status = custom_interfaces::msg::Heartbeat::RUNNING;
}

void FilterDetection::shutDownNode()
{
    status = custom_interfaces::msg::Heartbeat::STOPPING;

    status = custom_interfaces::msg::Heartbeat::STOPPED;
}

rclcpp_action::GoalResponse FilterDetection::_handleGoal(
    const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const FilterDetectionAction::Goal> goal)
{
    (void)goal;
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FilterDetection::_handleCancel(
    const std::shared_ptr<GoalHandleFilterDetection> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FilterDetection::_handleAccepted(const std::shared_ptr<GoalHandleFilterDetection> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&FilterDetection::_execute, this, std::placeholders::_1), goal_handle}.detach();
}

void FilterDetection::_execute(const std::shared_ptr<GoalHandleFilterDetection> goal_handle)
{
    auto result = std::make_shared<FilterDetectionAction::Result>();
    if (status != custom_interfaces::msg::Heartbeat::RUNNING)
    {
        RCLCPP_WARN_ONCE(this->get_logger(), "Node is not in running state");
        goal_handle->abort(result);
        return;
    }

    helpers::Timer timer("Filter detections action", get_logger());
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    if (!_detections_msg || _detections_msg->header.stamp == builtin_interfaces::msg::Time())
    {
        RCLCPP_ERROR(this->get_logger(), "There is no data form detectron - aborting.");
        _publisher->publish(custom_interfaces::msg::Detections());
        goal_handle->abort(result);
        return;
    }

    if (_last_processed_msg_timestamp == _detections_msg->header.stamp)
        RCLCPP_WARN(get_logger(), "New message has not arrived yet. Processing old message.");
    _last_processed_msg_timestamp = _detections_msg->header.stamp;

    int masks_ammount_cam1 = _detections_msg->cam1_masks.size();
    int masks_ammount_cam2 = _detections_msg->cam2_masks.size();

    RCLCPP_INFO(this->get_logger(), "Recived " + std::to_string(masks_ammount_cam1) + " masks for camera 1");
    RCLCPP_INFO(this->get_logger(), "Recived " + std::to_string(masks_ammount_cam2) + " masks for camera 2");

    if (_assignInputData(_detections_msg))
    {
        RCLCPP_ERROR(this->get_logger(), "data was not assigned properly.");
        _publisher->publish(custom_interfaces::msg::Detections());
        goal_handle->abort(result);
        return;
    }

    _handleMultipleDetections();
    RCLCPP_INFO(this->get_logger(), "Filtered " + std::to_string(masks_ammount_cam1 - _detections_msg->cam1_masks.size()) + " masks for camera 1");
    RCLCPP_INFO(this->get_logger(), "Filtered " + std::to_string(masks_ammount_cam2 - _detections_msg->cam2_masks.size()) + " masks for camera 2");

    //************************Containers code***********************************//
    // extract container masks from all detection masks
    auto findContainerMasks = [this](const std::vector<LabeledMasks> &detection_cam, const std::string &container_label, std::vector<cv::Mat> &out_labeled_container_masks) -> bool
    {
        auto labeled_container_masks = std::find_if(detection_cam.begin(), detection_cam.end(),
                                                    [this, container_label](const LabeledMasks &labeled_masks)
                                                    {
                                                        return container_label == labeled_masks.label;
                                                    });
        if (labeled_container_masks != detection_cam.end())
        {
            out_labeled_container_masks = labeled_container_masks->masks;
            RCLCPP_INFO_STREAM(this->get_logger(), "Found container " << container_label);
            return true;
        }
        else
            return false;
    };
    auto removeIntersection = [](cv::Mat &overlayed_container_mask, cv::Mat &label_mask) -> void
    {
        // exists overlaying
        cv::Mat label_mask_intersection;
        // find intersection b/w container and label mask
        cv::bitwise_and(label_mask, overlayed_container_mask, label_mask_intersection);
        // remove label mask intersection from container label
        overlayed_container_mask = overlayed_container_mask - label_mask_intersection;
    };
    // auto showImage = [](cv::Mat &mask, std::string mask_name) {
    //     cv::imshow(mask_name, mask);
    //     cv::waitKey(0);
    //     cv::destroyAllWindows();
    // };
    // auto handleIntersection = [this, &removeIntersection](LabeledMasks &detection_cam,std::string &container_label, std::map<uint32_t, std::vector<cv::Mat>> &out_labeled_containers_masks) {
    //     for (auto label_mask_it = detection_cam.masks.begin(); label_mask_it != detection_cam.masks.end(); label_mask_it++)
    //     {
    //         // showImage(*label_mask_it, std::to_string(label_mask_it - detection_cam.masks.begin()));
    //         auto overlayed_container_mask_it = std::find_if(out_labeled_containers_masks[_labels_str_to_int[container_label]].begin(),
    //                                                         out_labeled_containers_masks[_labels_str_to_int[container_label]].end(),
    //                                                         [this, label_mask_it](cv::Mat &container_mask) { return _checkOverlay(*label_mask_it, container_mask, 0.1); });
    //         if (overlayed_container_mask_it != out_labeled_containers_masks[_labels_str_to_int[container_label]].end())
    //         {
    //             RCLCPP_INFO(this->get_logger(), "found intersection");
    //             removeIntersection(*overlayed_container_mask_it, *label_mask_it);
    //         }
    //         else
    //         {
    //             RCLCPP_INFO(this->get_logger(), "No intersection");
    //         }
    //     }
    // };
    auto filterContainersMasksFromOverlayingMasks = [this, &removeIntersection](std::vector<LabeledMasks> &detections_cam, std::vector<std::string> &containers_labels, std::map<std::string, std::vector<cv::Mat>> &out_labeled_containers_masks) -> void
    {
        // for each label masks
        for (auto &detection_cam : detections_cam)
        {
            // skip the container
            if (out_labeled_containers_masks.find(detection_cam.label) != out_labeled_containers_masks.end())
                continue;
            // check against all containers
            for (auto &container_label : containers_labels)
            {
                // RCLCPP_INFO_STREAM(this->get_logger(), "checking " << detection_cam.label << " against " << container_label);
                // one label masks
                for (auto label_mask_it = detection_cam.masks.begin(); label_mask_it != detection_cam.masks.end(); label_mask_it++)
                {
                    // showImage(*label_mask_it, std::to_string(label_mask_it - detection_cam.masks.begin()));
                    auto overlayed_container_mask_it = std::find_if(out_labeled_containers_masks[container_label].begin(),
                                                                    out_labeled_containers_masks[container_label].end(),
                                                                    [this, label_mask_it](cv::Mat &container_mask)
                                                                    { return _checkOverlay(*label_mask_it, container_mask, 0.1); });
                    if (overlayed_container_mask_it != out_labeled_containers_masks[container_label].end())
                    {
                        RCLCPP_INFO(this->get_logger(), "found intersection");
                        removeIntersection(*overlayed_container_mask_it, *label_mask_it);
                    }
                    // else
                    // {
                    //     RCLCPP_INFO(this->get_logger(), "No intersection");
                    // }
                }
            }
        }
    };
    auto modifyDetectionMasks = [this](std::vector<LabeledMasks> &detections_cam, std::vector<std::string> &containers_labels, std::map<std::string, std::vector<cv::Mat>> &labeled_containers_masks) -> void
    {
        for (auto &container_label : containers_labels)
        {
            for (auto &detection_cam : detections_cam)
            {
                // modify the container
                if (detection_cam.label == container_label)
                {
                    RCLCPP_INFO_STREAM(this->get_logger(), "modity container " << container_label);
                    detection_cam.masks = labeled_containers_masks[container_label];
                }
                else // ignore all other masks
                    continue;
            }
        }
    };
    std::vector<std::string> containers_labels{"plate", "bowl", "cuttin_board"};
    std::map<std::string, std::vector<cv::Mat>> labeled_containers_masks_cam1;
    std::map<std::string, std::vector<cv::Mat>> labeled_containers_masks_cam2;

    for (auto container_label_iter = containers_labels.begin(); container_label_iter != containers_labels.end();)
    {
        bool container_cam1 = findContainerMasks(_detections_cam1, *container_label_iter, labeled_containers_masks_cam1[*container_label_iter]);
        bool container_cam2 = findContainerMasks(_detections_cam2, *container_label_iter, labeled_containers_masks_cam2[*container_label_iter]);
        if (!container_cam1 && !container_cam2)
        {
            container_label_iter = containers_labels.erase(container_label_iter);
        }
        else
            ++container_label_iter;
    }
    if (labeled_containers_masks_cam1.size() != 0)
    {
        filterContainersMasksFromOverlayingMasks(_detections_cam1, containers_labels, labeled_containers_masks_cam1);
        modifyDetectionMasks(_detections_cam1, containers_labels, labeled_containers_masks_cam1);
    }
    if (labeled_containers_masks_cam2.size() != 0)
    {
        filterContainersMasksFromOverlayingMasks(_detections_cam2, containers_labels, labeled_containers_masks_cam2);
        modifyDetectionMasks(_detections_cam2, containers_labels, labeled_containers_masks_cam2);
    }

    //************************Containers code***********************************//

    //TODO this part is not finished yet therefore removed from pipeline do not uncomment yet or it will crash the service
    // if (!_reconstructMissingMasks())
    // {
    //     RCLCPP_ERROR(this->get_logger(), "Error encounter when trying to reconstruct masks.");
    //     _publisher->publish(custom_interfaces::msg::Detect());
    //     goal_handle->abort(result);
    //     return;
    // }

    custom_interfaces::msg::Detections::UniquePtr out_detect_msg(new custom_interfaces::msg::Detections);
    _assignFilteredData(out_detect_msg);

    out_detect_msg->header.stamp = now();
    _publisher->publish(std::move(out_detect_msg));

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

int FilterDetection::_findUncompleteItems(std::vector<LabeledMasks> &out_wrappers, std::vector<LabeledMasks> &out_components)
{
    int dupa = 0;
    std::vector<LabeledMasks> matched_wrappers;
    std::vector<LabeledMasks> matched_components;
    std::cout << "_findUncompleteItems " << ++dupa << std::endl;

    auto wrapper = out_wrappers.end();
    while (wrapper != out_wrappers.begin())
    {
        wrapper--;
        std::cout << "wrapper " << ++dupa << std::endl;

        for (auto elem : _components)
        {
            std::cout << elem.first << " " << elem.first << " " << elem.second.size() << "\n";
        }

        size_t expected_elements_ammount = _components[wrapper->label].size();
        auto wrap_mask = wrapper->masks.end();
        while (wrap_mask != wrapper->masks.begin())
        {
            wrap_mask--;
            std::cout << "wrap_mask" << ++dupa << std::endl;

            std::vector<LabeledMasks> matched_elements;

            auto possible_components = out_components.end();
            while (possible_components != out_components.begin())
            {
                possible_components--;
                std::cout << "possible_components" << ++dupa << std::endl;

                if (_components[wrapper->label].find(possible_components->label) != _components[wrapper->label].end())
                {
                    auto el_mask = possible_components->masks.end();
                    while (el_mask != possible_components->masks.begin())
                    {
                        el_mask--;
                        std::cout << "el_mask" << ++dupa << std::endl;

                        if (_checkOverlay(*wrap_mask, *el_mask))
                        {
                            LabeledMasks matched_component;
                            matched_component.label = possible_components->label;
                            matched_component.masks.push_back(*el_mask);
                            possible_components->masks.erase(el_mask);
                            matched_elements.push_back(matched_component);
                            //break here becouse we EXPECT to have only one mask of each element after filtration duplicates.
                            break;
                        }
                    }
                }
            }

            if (matched_elements.size() != expected_elements_ammount)
                _genereteMissingComponents(*wrap_mask, wrapper->label, matched_elements);

            if (matched_elements.size() == expected_elements_ammount)
            {
                auto matched_wrappers_it = std::find_if(matched_wrappers.begin(), matched_wrappers.end(), [wrapper](LabeledMasks &current_wrapper)
                                                        { return (wrapper->label == current_wrapper.label); });
                if (matched_wrappers_it == matched_wrappers.end())
                {
                    LabeledMasks temp_wrapper;
                    temp_wrapper.label = wrapper->label;
                    temp_wrapper.masks.push_back(*wrap_mask);
                    matched_wrappers.push_back(temp_wrapper);
                }
                else
                    matched_wrappers_it->masks.push_back(*wrap_mask);

                for (auto matched_element : matched_elements)
                {
                    auto matched_component_it = std::find_if(matched_components.begin(), matched_components.end(), [matched_element](LabeledMasks current_component)
                                                             { return (matched_element.label == current_component.label); });
                    if (matched_component_it == matched_components.end())
                        matched_components.push_back(matched_element);
                    else
                    {
                        matched_component_it->masks.reserve(matched_component_it->masks.size() + matched_element.masks.size());
                        matched_component_it->masks.insert(matched_component_it->masks.end(), matched_element.masks.begin(), matched_element.masks.end());
                    }
                }
                wrapper->masks.erase(wrap_mask);
            }
        }
    }
    return 0;
}

int FilterDetection::_genereteMissingComponents(cv::Mat &wrapper, std::string wrapper_label, std::vector<LabeledMasks> &out_matched_elements)
{
    if (_components[wrapper_label].size() > out_matched_elements.size() + 1)
    {
        RCLCPP_INFO(this->get_logger(), "reconstructing components for " + wrapper_label + " not possible - to much missing data.");
        return 1;
    }
    LabeledMasks missing_label;
    for (auto component : _components[wrapper_label])
    {
        auto el = std::find_if(out_matched_elements.begin(), out_matched_elements.end(), [component](const LabeledMasks &current_element)
                               { return component == current_element.label; });
        if (el == out_matched_elements.end())
        {
            missing_label.label = component;
            break;
        }
    }
    cv::Mat missing_mask;
    for (auto element : out_matched_elements)
    {
        cv::Mat reversed_element_mask = 1 - element.masks[0];
        cv::bitwise_or(wrapper, reversed_element_mask, missing_mask);
    }

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(missing_mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Mat> generated_masks;
    for (size_t i = 0; i < contours.size(); i++)
    {
        cv::Mat mask = cv::Mat::zeros(missing_mask.size(), CV_8UC1);
        cv::drawContours(mask, contours[i], 0, 255, 2, -1);
        generated_masks.push_back(mask);
    }
    cv::Mat biggest_mask = cv::Mat::zeros(missing_mask.size(), CV_8UC1);
    ;
    for (auto &mask : generated_masks)
    {
        if (cv::countNonZero(biggest_mask) < cv::countNonZero(mask))
            biggest_mask = mask;
    }

    // cv::namedWindow("title", 1);
    // cv::imshow("image1", biggest_mask);
    // cv::waitKey(0);
    // cv::destroyAllWindows();

    return 0;
}

int FilterDetection::_reconstructMissingMasks()
{
    std::vector<LabeledMasks> wrappers;
    std::vector<LabeledMasks> components;

    auto assign_data = [this](std::vector<LabeledMasks> &detections, std::vector<LabeledMasks> &wrappers, std::vector<LabeledMasks> &components) mutable
    {
        wrappers.clear();
        components.clear();
        std::vector<LabeledMasks> noncompound;
        for (auto &labeles_masks : detections)
        {
            if (_items[labeles_masks.label] && _elements[labeles_masks.label])
                noncompound.push_back(labeles_masks);
            else if (_items[labeles_masks.label])
                wrappers.push_back(labeles_masks);
            else if (_elements[labeles_masks.label])
                components.push_back(labeles_masks);
            else
            {
                RCLCPP_ERROR(this->get_logger(), "elements cant be categorized. aborting!");
                return 1;
            }
        }
        detections.clear();
        for (auto &labeles_masks : noncompound)
            detections.push_back(labeles_masks);
        return 0;
    };

    //camera 1
    if (assign_data(_detections_cam1, wrappers, components))
        return 1;

    std::cout << wrappers.size() << std::endl;
    std::cout << components.size() << std::endl;
    std::cout << _detections_cam1.size() << std::endl;
    _findUncompleteItems(wrappers, components);
    //camera 2
    if (assign_data(_detections_cam2, wrappers, components))
        return 1;
    _findUncompleteItems(wrappers, components);
    return 0;
}

int FilterDetection::_readLabels()
{
    json labels_params = helpers::commons::getParameter("labels");
    for (auto label : labels_params)
    {
        json label_data = label;
        if (label_data.contains("label") && label_data.contains("item") && label_data.contains("element"))
        {
            bool item = label_data["item"].get<bool>();
            _items[label_data["label"]] = item;
            bool element = label_data["element"].get<bool>();
            _elements[label_data["label"]] = element;

            if (!element)
            {
                std::vector<std::string> components = label_data["components"];
                std::set<std::string> components_str;
                for (auto component : components)
                {
                    components_str.insert(component);
                }
                _components[label_data["label"]] = components_str;
            }
        }
    }

    return 0;
}

int FilterDetection::_assignFilteredData(custom_interfaces::msg::Detections::UniquePtr &detect_msg)
{
    auto assign_data = [this](std::vector<LabeledMasks> &detections, std::vector<std::string> &out_masks, std::vector<std::string> &out_labels) mutable
    {
        out_masks.clear();
        out_labels.clear();
        for (auto labeled_masks : detections)
        {
            for (auto mask : labeled_masks.masks)
            {
                out_labels.push_back(labeled_masks.label);
                std::string mask_string;
                helpers::converters::binaryMaskToString(mask, mask_string);
                out_masks.push_back(mask_string);
            }
        }
    };

    // camera 1
    assign_data(_detections_cam1, detect_msg->cam1_masks, detect_msg->cam1_labels);
    // camera 2
    assign_data(_detections_cam2, detect_msg->cam2_masks, detect_msg->cam2_labels);
    return 0;
}

void FilterDetection::prepareDetectionVectors(custom_interfaces::msg::Detections::SharedPtr &detect_msg)
{
    _detections_cam1.clear();
    _detections_cam2.clear();
    std::set<std::string> unique_labels_cam1;
    std::set<std::string> unique_labels_cam2;
    for (auto label : detect_msg->cam1_labels)
        unique_labels_cam1.insert(label);
    for (auto label : detect_msg->cam2_labels)
        unique_labels_cam2.insert(label);
    _detections_cam1.resize(unique_labels_cam1.size());
    _detections_cam2.resize(unique_labels_cam2.size());
}

int FilterDetection::_assignInputData(custom_interfaces::msg::Detections::SharedPtr &detect_msg)
{
    if (detect_msg->cam1_masks.size() != detect_msg->cam1_labels.size() || detect_msg->cam2_masks.size() != detect_msg->cam2_labels.size())
    {
        RCLCPP_ERROR(this->get_logger(), "Detectron data incorrect - number of mask does not match numer of labels.");
        return 1;
    }
    prepareDetectionVectors(detect_msg);

    auto assign_data = [this](std::vector<cv::String> &masks, std::vector<std::string> &labels, std::vector<LabeledMasks> &labeled_masks) mutable
    {
        for (size_t i = 0; i < masks.size(); i++)
        {
            std::string current_label = labels[i];
            std::vector<LabeledMasks>::iterator labeled_mask_it = std::find_if(labeled_masks.begin(), labeled_masks.end(), [current_label](const LabeledMasks &labeled_mask)
                                                                               { return (labeled_mask.label == current_label || labeled_mask.label.empty()); });
            if (labeled_mask_it == labeled_masks.end())
            {
                RCLCPP_ERROR(this->get_logger(), "Data was not assign properly.");
                return 1;
            }
            cv::Mat current_mask;
            helpers::converters::stringToBinaryMask(masks[i], current_mask);
            labeled_mask_it->label = current_label;
            labeled_mask_it->masks.push_back(current_mask);
        }
        return 0;
    };

    //camera 1
    if (assign_data(detect_msg->cam1_masks, detect_msg->cam1_labels, _detections_cam1))
        return 1;
    //camera 2
    if (assign_data(detect_msg->cam2_masks, detect_msg->cam2_labels, _detections_cam2))
        return 1;

    return 0;
}

bool FilterDetection::_checkOverlay(cv::Mat &mask1, cv::Mat &mask2, float overlay_margin)
{
    if (mask1.empty() || mask2.empty())
        return false;

    std::vector<int> sizes{cv::countNonZero(mask1), cv::countNonZero(mask2)};
    auto [min, max] = std::minmax_element(sizes.begin(), sizes.end());
    cv::Mat merged;
    cv::bitwise_and(mask1, mask2, merged);
    int size_merged = cv::countNonZero(merged);
    if (size_merged >= (1.0 - overlay_margin) * *min && size_merged <= (1.0 + overlay_margin) * *max)
        return true;
    return false;
}

bool FilterDetection::_checkOverlay(cv::Mat &item_mask, cv::Mat &element_mask)
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

int FilterDetection::_handleMultipleDetections()
{
    helpers::Timer timer("Removing duplicated masks.", true);
    auto merge_overlaping_masks = [this](std::vector<LabeledMasks> &labeled_masks) mutable
    {
        for (auto &detections : labeled_masks)
        {
            for (auto it = detections.masks.begin(); it != detections.masks.end(); it++)
            {
                if (std::distance(it, detections.masks.end()) <= 0)
                    break;
                std::vector<cv::Mat>::iterator overlayed_mask_it = std::find_if(it + 1, detections.masks.end(), [this, it](cv::Mat &mask)
                                                                                { return _checkOverlay(*it, mask, 0.1); });
                if (std::distance(overlayed_mask_it, detections.masks.end()) <= 0)
                    continue;

                if (_debug)
                {
                    cv::namedWindow("title", 1);
                    cv::imshow("image1", *it);
                    cv::imshow("image2", *overlayed_mask_it);
                    cv::waitKey(0);
                    cv::destroyAllWindows();
                }

                cv::bitwise_or(*it, *overlayed_mask_it, *overlayed_mask_it);
                detections.masks.erase(it);
            }
        }
    };

    //camera 1
    merge_overlaping_masks(_detections_cam1);
    //camera 2
    merge_overlaping_masks(_detections_cam2);
    return 0;
}

FilterDetection::~FilterDetection() {}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(FilterDetection)
