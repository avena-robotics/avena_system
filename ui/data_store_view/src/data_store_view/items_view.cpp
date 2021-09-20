#include <data_store_view/items_view.h>

namespace data_store_view
{
    ItemsView::ItemsView(Ui::DataStoreView *ui_ptr, std::shared_ptr<rclcpp::Node> node_shared_ptr, QWidget *widget, size_t amount_of_cameras)
        : _ui_ptr(ui_ptr), _node_shared_ptr(node_shared_ptr), _widget(widget), _cameras_amount(amount_of_cameras), _current_camera(0)
    {
        qRegisterMetaType<custom_interfaces::msg::ItemElement>("custom_interfaces::msg::ItemElement");

        connectSlotsToSignals();
        _scene_shared_ptr = std::make_shared<QGraphicsScene>();
        _scene_shared_ptr->setSceneRect(0, 0, 1270, 720);
        _ui_ptr->maskView->setScene(_scene_shared_ptr.get());
        _current_item_ptr = nullptr;
        _current_item_element = nullptr;
        _back_element_id = 0;
        std::string share_directory_path = ament_index_cpp::get_package_share_directory("data_store_view");
        _rviz_config_path = share_directory_path + "/rviz_configs/all_cameras_pointclouds.rviz";
        std::string no_mask_image_path = share_directory_path + "/assets/no_mask.png";
        _rviz_pid = -1;

        _no_mask_image = cv::imread(no_mask_image_path);
        if(_no_mask_image.empty())
        {
            throw error_str("cannot load no mask image: " + no_mask_image_path);
        }

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        for(size_t i=0; i<_cameras_amount; i++)
        {
            _publishers.push_back(_node_shared_ptr->create_publisher<sensor_msgs::msg::PointCloud2>(
                "camera_" + std::to_string(i+1) + "/element",
                qos
            ));
        }

    }

    ItemsView::~ItemsView()
    {
        _current_item_ptr = nullptr;
        _current_item_element = nullptr;
    }

    void ItemsView::showMaskView()
    {
        _ui_ptr->stackedWidget->setCurrentIndex(static_cast<int>(ViewIndex::IMAGES));
    }
    void ItemsView::showItemsTab()
    {
        _ui_ptr->stackedWidget->setCurrentIndex(static_cast<int>(ViewIndex::ITEMS));
    }
    void ItemsView::showElementsTab(int idx)
    {
        if (idx > _items.items.size() - 1)
        {
            throw error_str("Invalid item id: " + std::to_string(idx) + " max is: " + std::to_string(_items.items.size()));
        }

        _current_item_ptr = &_items.items[idx];

        if (_current_item_ptr == nullptr)
        {
            throw error_str("Item is nullptr");
        }

        _ui_ptr->stackedWidget->setCurrentIndex(static_cast<int>(ViewIndex::ELEMENTS));
        _ui_ptr->itemDescription->setText("ITEM: " + QString::number(_current_item_ptr->id) + " " + QString(_current_item_ptr->label.c_str()));
        uint32_t row_idx = 0;
        _ui_ptr->elementsTable->setRowCount(_current_item_ptr->item_elements.size());
        for (auto element : _current_item_ptr->item_elements)
        {
            std::cout << element.label << std::endl;
            _ui_ptr->elementsTable->setItem(row_idx, static_cast<int>(ElementsTableColumns::ID), new QTableWidgetItem(QString::number(element.id)));
            _ui_ptr->elementsTable->setItem(row_idx, static_cast<int>(ElementsTableColumns::LABEL), new QTableWidgetItem(QString(element.label.c_str())));
            _ui_ptr->elementsTable->setItem(row_idx, static_cast<int>(ElementsTableColumns::POINT_CLOUDS), new QTableWidgetItem(QString(element.label.c_str())));

            auto pcld_button = new QPushButton();
            pcld_button->setText("Show PointClouds");
            connect(pcld_button, &QPushButton::clicked, this, 
                [=](){
                    showPointCloudsInRviz(element); 
                });
            _ui_ptr->elementsTable->setCellWidget(row_idx, static_cast<int>(ElementsTableColumns::POINT_CLOUDS), pcld_button);

            auto masks_button = new QPushButton();
            masks_button->setText("Show Masks");
            connect(masks_button, &QPushButton::clicked, this,
                    [=]()
                    {
                        
                        _current_item_element = const_cast<custom_interfaces::msg::ItemElement*>(&element);
                        _back_element_id = idx;
                        fillCamerasComboBox();
                        if (element.masks[_current_camera].size() > 0)
                            showImage(element.masks[_current_camera]);
                        else{
                            showImage(_no_mask_image);
                            std::cout << "Mask is empty" << std::endl;
                        }
                    });
            _ui_ptr->elementsTable->setCellWidget(row_idx, static_cast<int>(ElementsTableColumns::MASKS), masks_button);
            row_idx++;
        }
    }

    void ItemsView::showMaskFromViewPort(const QString& camera_name)
    {
        std::string camera_name_str = camera_name.toStdString();

        if(camera_name.size() == 0)
        {
            showImage(_no_mask_image);
        }

        try
        {
            /* code */
            _current_camera  = std::stoi(camera_name_str.substr(7,1))-1;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cout << "current camera: " << _current_camera << std::endl;
        }

        if(_current_camera>=9)
            throw error_str("Invalid camera index");

        showImage(_current_item_element->masks[_current_camera]);
    }

    void ItemsView::showMasksOfCurrentElement()
    {
        if (_current_item_element == nullptr)
            return;
    }

    void ItemsView::showPointCloudsInRviz(const custom_interfaces::msg::ItemElement& item_element)
    {
        uint32_t no_subs = _node_shared_ptr->count_subscribers("camera_9/element");

        std::cout << "no_subs: " << no_subs << std::endl;
        
        if(0 != kill(_rviz_pid, 0) || _rviz_pid == -1 )
        {
            std::cout << "Running rviz" << std::endl;
            QProcess rviz_process;
            rviz_process.setProgram("rviz2");
            rviz_process.setArguments({"-d", QString(_rviz_config_path.c_str())});
            rviz_process.startDetached(&_rviz_pid);

            if(_rviz_pid <= 0)
                throw error_str("Rviz failed");

            // size_t dupa = 0;
            // while (dupa <= no_subs){
            //     QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
            //     std::cout << dupa << std::endl;
            //     dupa = _node_shared_ptr->count_subscribers("camera_9/element");
            // }
            // delay(2000);
            // std::cout << no_subs << std::endl;
        }

        
        std::cout << "Publishing Pointcloud" << std::endl;
        if(item_element.ptclds.size() != _cameras_amount)
            throw error_str("Wrong amounts of pointclouds");
        for(size_t i=0; i<item_element.ptclds.size(); i++)
        {
            std::cout << "PointCloud: " << item_element.ptclds[i].data.size() << " " << item_element.ptclds[i].header.frame_id << std::endl;
            if(item_element.ptclds[i].data.size() == 0)
                continue;
            _publishers[i]->publish(item_element.ptclds[i]);
        }
    }

    void ItemsView::fillCamerasComboBox()
    {
        _ui_ptr->chooseDetectronViewpointComboBox->clear();
        for (size_t i = 1; i <= _cameras_amount; i++)
        {
            _ui_ptr->chooseDetectronViewpointComboBox->addItem(QString("camera_") + QString::number(i));
        }
    }

    void ItemsView::dataStoreInsertCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        emit datastoreUpdated();
        emit itemsUpdated();
    }

    void ItemsView::connectSlotsToSignals()
    {
        connect(this, SIGNAL(itemsUpdated()), this, SLOT(update()));
        connect(_ui_ptr->backButton_2, SIGNAL(clicked()), this, SLOT(showItemsTab()));
        connect(this, SIGNAL(datastoreUpdated()), this, SLOT(getItemsData()));
        connect(_ui_ptr->backButton, &QPushButton::clicked, this, [=](){
            showElementsTab(_back_element_id);
        });
        connect(_ui_ptr->chooseDetectronViewpointComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(showMaskFromViewPort(const QString&)));
    }

    void ItemsView::showImage(const std::string &mask_string)
    {
        if(mask_string.size() <= 0)
        {
            showImage(_no_mask_image);
            std::cout << "Mask is empty" << std::endl;
            return;
        }
        std::cout << mask_string << std::endl;
        cv::Mat mat;
        helpers::converters::stringToBinaryMask(mask_string, mat);
        std::cout << "Mat type: " << mat.type() << std::endl;
        cv::Mat rgb_mat;
        cv::cvtColor(mat, rgb_mat, cv::COLOR_GRAY2RGBA);
        showMaskView();

        QImage qt_image = QImage(rgb_mat.data, rgb_mat.cols, rgb_mat.rows, QImage::Format_RGBA8888);
        _scene_shared_ptr->addPixmap(QPixmap::fromImage(qt_image));

        _ui_ptr->maskView->show();
    }

    void ItemsView::showImage(const cv::Mat& image)
    {
        cv::Mat rgba_image;
        cv::cvtColor(image,rgba_image, cv::COLOR_RGB2RGBA);
        showMaskView();
        QImage qt_image = QImage(rgba_image.data, rgba_image.cols, rgba_image.rows, QImage::Format_RGBA8888);
        _scene_shared_ptr->addPixmap(QPixmap::fromImage(qt_image));
        _ui_ptr->maskView->show();
    }

    void ItemsView::getItemsData()
    {
        using namespace std::chrono_literals;

        auto items_request = std::make_shared<custom_interfaces::srv::DataStoreItemsSelect::Request>();
        if (!_data_store_select_client->wait_for_service(1s))
            throw error_str("failed while waiting for service");

        auto items_result = _data_store_select_client->async_send_request(items_request);
        // Wait for the result.
        if (items_result.wait_for(5s) == std::future_status::ready)
            _items = items_result.get()->data;
        else
            throw error_str("Failed to read compose items data");

        for (auto &item : _items.items)
        {
            std::cout << item.label << std::endl;
        }
    }

    void ItemsView::testRows(uint32_t &row)
    {
        _ui_ptr->itemsTable->setRowCount(1);
        auto label = new QLabel();
        geometry_msgs::msg::Pose pose;
        pose.position.x = 1;
        pose.position.y = 2;
        pose.position.z = 3;
        pose.orientation.x = 4;
        pose.orientation.y = 5;
        pose.orientation.z = 6;
        pose.orientation.w = 7;
        label->setText(geometryMsgPoseToString(pose).c_str());
        _ui_ptr->itemsTable->setRowHeight(row, 200);
        _ui_ptr->itemsTable->setCellWidget(row, 3, label);
    }

    void ItemsView::fillPoseField(const geometry_msgs::msg::Pose &pose, const uint32_t &row)
    {
        auto label = new QLabel();
        label->setText(geometryMsgPoseToString(pose).c_str());
        _ui_ptr->itemsTable->setRowHeight(row, 200);
        _ui_ptr->itemsTable->setCellWidget(row, static_cast<int>(ItemTableColumns::POSE), label);
    }

    void ItemsView::fillElementsButtonField(const uint32_t &row)
    {
        auto button = new QPushButton();
        button->setText("Show Elements");
        connect(button, &QPushButton::clicked, this, [=]()
                { showElementsTab(row); });
        _ui_ptr->itemsTable->setCellWidget(row, static_cast<int>(ItemTableColumns::ELEMENTS), button);
    }

    void ItemsView::update()
    {
        _ui_ptr->itemsTable->setRowCount(_items.items.size());
        uint32_t row_idx = 0;

        for (auto &item : _items.items)
        {
            fillRowWithItemInfo(item, row_idx);
            row_idx++;
        }
    }

    void ItemsView::fillRowWithItemInfo(custom_interfaces::msg::Item &item, uint32_t &row)
    {
        _ui_ptr->itemsTable->setItem(row, static_cast<int>(ItemTableColumns::ID), new QTableWidgetItem(QString::number(item.id)));
        _ui_ptr->itemsTable->setItem(row, static_cast<int>(ItemTableColumns::TYPE), new QTableWidgetItem(QString("Item")));
        _ui_ptr->itemsTable->setItem(row, static_cast<int>(ItemTableColumns::LABEL), new QTableWidgetItem(QString(item.label.c_str())));
        fillPoseField(item.pose, row);
        fillElementsButtonField(row);
    }

    void ItemsView::setUp()
    {
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        _data_store_changes_sub = _node_shared_ptr->create_subscription<std_msgs::msg::String>(
            "items_change_flag", qos_settings, std::bind(&ItemsView::dataStoreInsertCallback, this, std::placeholders::_1));
        _data_store_select_client = _node_shared_ptr->create_client<custom_interfaces::srv::DataStoreItemsSelect>("items_select");
    }
}