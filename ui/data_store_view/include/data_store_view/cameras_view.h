#pragma once

#include <ui_data_store_view.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <helpers_commons/helpers_commons.hpp>
#include <custom_interfaces/msg/rgbd_sync.hpp>
#include <std_msgs/msg/string.hpp>
#include <QImage>
#include <custom_interfaces/srv/data_store_rgbd_sync_select.hpp>
#include <helpers_vision/helpers_vision.hpp>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <data_store_view/utils.h>
#include <QWidget>

#define debug() debug_map.find(__func__) == debug_map.end() ? debug_map[__func__] = 1 : debug_map[__func__]++; std::cout << __func__ << " " << debug_map[__func__] << std::endl
#define error_str(x) std::runtime_error(std::string("\n\tLine: " + std::to_string(__LINE__) + "\n\tFile: " + __FILE__ + "\n\tFunc: " + __func__ + "\n\tMessage: " + x))

namespace data_store_view
{
    class CamerasView : public QObject
    {
        Q_OBJECT

        Ui::DataStoreView* _ui_ptr;
        std::shared_ptr<rclcpp::Node> _node_shared_ptr;
        size_t _current_camera;
        rclcpp::Client<custom_interfaces::srv::DataStoreRgbdSyncSelect>::SharedPtr _rgbd_sync_select_client;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _data_store_changes_sub;
        std::map<size_t, QImage> _images;
        size_t _cameras_amount;
        custom_interfaces::msg::RgbdSync _rgbd_sync;
        std::shared_ptr<QGraphicsScene> _scene_shared_ptr;

        void fillCamerasComboBox();
        void connectSlotsToSignals();

        void displayImage(size_t index);
        void dataStoreInsertCallback(const std_msgs::msg::String::SharedPtr msg);

    signals:
        void imageUpdated();
        void datastoreUpdated();

    private slots:
        virtual void getImages();
        virtual void update();
        virtual void changeCamera(const QString& value);

    public:
        CamerasView(Ui::DataStoreView* ui, std::shared_ptr<rclcpp::Node> node, size_t amount_of_cameras);
        ~CamerasView();

        void setUp();


        std::map<std::string, size_t> debug_map;
    };
}