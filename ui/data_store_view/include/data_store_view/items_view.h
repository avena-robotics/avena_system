#pragma once

#include <QObject>
#include <ui_data_store_view.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <custom_interfaces/srv/data_store_items_select.hpp>
#include <std_msgs/msg/string.hpp>
#include <QTimer>
#include <QGraphicsPixmapItem>
#include <QMessageBox>
#include <data_store_view/utils.h>
#include <QJsonDocument>
#include <QFile>
#include <helpers_vision/helpers_vision.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <QProcess>
#include <data_store_view/utils.h>

#define debug() debug_map.find(__func__) == debug_map.end() ? debug_map[__func__] = 1 : debug_map[__func__]++; std::cout << __func__ << " " << debug_map[__func__] << std::endl
#define error_str(x) std::runtime_error(std::string("\n\tLine: " + std::to_string(__LINE__) + "\n\tFile: " + __FILE__ + "\n\tFunc: " + __func__ + "\n\tMessage: " + x))

Q_DECLARE_METATYPE(custom_interfaces::msg::ItemElement)

namespace data_store_view
{

    enum class ItemTableColumns
    {
        ID,
        TYPE,
        LABEL,
        POSE,
        ELEMENTS
    };

    enum class ElementsTableColumns
    {
        ID,
        LABEL,
        POINT_CLOUDS,
        SHADOW_POINTS,
        MASKS,
        PART_DESCRIPTION
    };

    enum class ViewIndex
    {
        ITEMS,
        ELEMENTS,
        IMAGES
    };

    class ItemsView : public QObject
    {
        Q_OBJECT

        Ui::DataStoreView* _ui_ptr;
        std::shared_ptr<rclcpp::Node> _node_shared_ptr;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _data_store_changes_sub;
        rclcpp::Client<custom_interfaces::srv::DataStoreItemsSelect>::SharedPtr _data_store_select_client;
        custom_interfaces::msg::Items _items;
        std::shared_ptr<QGraphicsScene> _scene_shared_ptr;
        QWidget* _widget;
        custom_interfaces::msg::Item* _current_item_ptr;
        custom_interfaces::msg::ItemElement* _current_item_element;
        size_t _cameras_amount;
        size_t _current_camera;
        int _back_element_id;
        cv::Mat _no_mask_image;
        std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _publishers;
        std::string _rviz_config_path;
        qint64 _rviz_pid;

        void dataStoreInsertCallback(const std_msgs::msg::String::SharedPtr msg);
        void connectSlotsToSignals();
        void fillRowWithItemInfo(custom_interfaces::msg::Item& item, uint32_t& row);
        void testRows(uint32_t& row);
        void fillPoseField(const geometry_msgs::msg::Pose& pose, const uint32_t& row);
        void fillElementsButtonField(const uint32_t& row);
        void fillCamerasComboBox();

    private slots:
        virtual void getItemsData();
        virtual void update();
        virtual void showImage(const std::string& mask_string);
        virtual void showImage(const cv::Mat& image);
        virtual void showMaskView();
        virtual void showItemsTab();
        virtual void showElementsTab(int);
        virtual void showPointCloudsInRviz(const custom_interfaces::msg::ItemElement& item_element);
        virtual void showMasksOfCurrentElement();
        virtual void showMaskFromViewPort(const QString& camera_name);
    signals:
        void itemsUpdated();
        void datastoreUpdated();
    
    public:
        ItemsView(Ui::DataStoreView* ui_ptr, std::shared_ptr<rclcpp::Node> node_shared_ptr, QWidget* widget, size_t amount_of_cameras);
        ~ItemsView();
        void setUp();


        std::map<std::string, size_t> debug_map;

    };
}