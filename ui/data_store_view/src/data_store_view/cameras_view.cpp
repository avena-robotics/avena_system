#include "data_store_view/cameras_view.h"

namespace data_store_view
{
    CamerasView::CamerasView(Ui::DataStoreView *ui, std::shared_ptr<rclcpp::Node> node, size_t amount_of_cameras)
        : _ui_ptr(ui), _node_shared_ptr(node), _cameras_amount(amount_of_cameras), _current_camera(1)
    {
        connectSlotsToSignals();
        _scene_shared_ptr = std::make_shared<QGraphicsScene>();
        _ui_ptr->cameraImageView->setScene(_scene_shared_ptr.get());
        fillCamerasComboBox();
    }

    CamerasView::~CamerasView()
    {
        std::cout << "I'm dying" << std::endl;
    }

    void CamerasView::setUp()
    {
        debug();
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        debug();
        _rgbd_sync_select_client = _node_shared_ptr->create_client<custom_interfaces::srv::DataStoreRgbdSyncSelect>("rgbd_sync_select");
        _data_store_changes_sub = _node_shared_ptr->create_subscription<std_msgs::msg::String>(
            "rgbd_sync_change_flag", qos_settings, std::bind(&CamerasView::dataStoreInsertCallback, this, std::placeholders::_1));
    }

    void CamerasView::fillCamerasComboBox()
    {
        _ui_ptr->chooseCameraComboBox->clear();
        for (size_t i = 1; i <= _cameras_amount; i++)
        {
            _ui_ptr->chooseCameraComboBox->addItem(QString("camera_") + QString::number(i));
        }
    }

    void CamerasView::getImages()
    {
        using namespace std::chrono_literals;
        auto rgbd_sync_request = std::make_shared<custom_interfaces::srv::DataStoreRgbdSyncSelect::Request>();
        if (!_rgbd_sync_select_client->wait_for_service(1s))
        {
            throw error_str("failed while waiting for service");
        }

        auto rgbd_sync_result = _rgbd_sync_select_client->async_send_request(rgbd_sync_request);
        // Wait for the result.
        auto rgbd_sync_future_result = rgbd_sync_result.wait_for(5s);

        debug();
        std::cout << "get images: " << static_cast<int>( rgbd_sync_future_result) << std::endl;
        if (rgbd_sync_future_result == std::future_status::ready)
        {   
            debug();
            _rgbd_sync = rgbd_sync_result.get()->data;
            debug();
        }
        else
        {
            throw error_str("failed to read rgbd data");
            // RCLCPP_ERROR(this->get_logger(), "Failed to read detectron data");
        }
    }

    void CamerasView::update()
    {
        if (_current_camera < 0 || _current_camera >= _cameras_amount)
            return;
        if (_rgbd_sync.rgbs.size() <= 0)
            return;
        sensor_msgs::msg::Image image = _rgbd_sync.rgbs[_current_camera];
        QImage qt_image;
        rosImageToQt(image, qt_image);
        _scene_shared_ptr->addPixmap(QPixmap::fromImage(qt_image));
        _ui_ptr->cameraImageView->show();
    }

    void CamerasView::connectSlotsToSignals()
    {
        connect(_ui_ptr->chooseCameraComboBox, SIGNAL(currentTextChanged(const QString &)), this, SLOT(changeCamera(const QString &)));
        // connect(_ui_ptr->refreshButton, SIGNAL(clicked(bool)), this, SLOT(getAmountOfCameras()));
        connect(this, SIGNAL(imageUpdated()), this, SLOT(update()));
        connect(this, SIGNAL(datastoreUpdated()), this, SLOT(getImages()));
        // connect(_ui_ptr->chooseCameraComboBox, SIGNAL(currentTextChanged(const QString &)))
    }

    void CamerasView::changeCamera(const QString &msg)
    {
        if (msg.size() <= 0)
        {
            return;
            _current_camera = -1;
        }
        std::string msg_str = msg.toStdString();
        std::string camera_id_str = msg_str.substr(7, 1);
        _current_camera = std::stoi(camera_id_str) - 1;
        std::cout << "Current view id: " << _current_camera << std::endl;
        update();
    }

    void CamerasView::dataStoreInsertCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        debug();
        emit datastoreUpdated();//saves images into _rgbd_sync
        debug();
        emit imageUpdated();
        debug();
    }
}