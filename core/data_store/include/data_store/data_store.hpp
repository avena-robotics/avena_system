#ifndef _DATA_STORE_HPP
#define _DATA_STORE_HPP

#include "helpers_commons/helpers_commons.hpp"
#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include "visibility_control.h"
// ___Avena___
#include "custom_interfaces/srv/data_store_rgb_data_insert.hpp"
#include "custom_interfaces/srv/data_store_rgb_data_delete.hpp"
#include "custom_interfaces/srv/data_store_rgb_data_select.hpp"
#include "custom_interfaces/srv/data_store_rgb_data_get_list.hpp"

#include "custom_interfaces/srv/data_store_item_cam1_insert.hpp"
#include "custom_interfaces/srv/data_store_item_cam1_delete.hpp"
#include "custom_interfaces/srv/data_store_item_cam1_select.hpp"
#include "custom_interfaces/srv/data_store_item_cam1_get_list.hpp"

#include "custom_interfaces/srv/data_store_item_cam2_insert.hpp"
#include "custom_interfaces/srv/data_store_item_cam2_delete.hpp"
#include "custom_interfaces/srv/data_store_item_cam2_select.hpp"
#include "custom_interfaces/srv/data_store_item_cam2_get_list.hpp"

#include <optional>

using Milliseconds = std::chrono::milliseconds;
namespace data_store
{
    enum class DataState_e : uint8_t
    {
        VALID = 0,
        EMPTY, // len ==0
        INVALID_HEADER,
        UNEXPECTED // not match expected data struct
    };
    class DataStore : public rclcpp::Node, public helpers::WatchdogInterface
    {

    public:
        using RgbDataInsert = custom_interfaces::srv::DataStoreRgbDataInsert;
        using RgbDataDelete = custom_interfaces::srv::DataStoreRgbDataDelete;
        using RgbDataSelect = custom_interfaces::srv::DataStoreRgbDataSelect;
        using RgbDataGetList = custom_interfaces::srv::DataStoreRgbDataGetList;

        using ItemCam1Insert = custom_interfaces::srv::DataStoreItemCam1Insert;
        using ItemCam1Delete = custom_interfaces::srv::DataStoreItemCam1Delete;
        using ItemCam1Select = custom_interfaces::srv::DataStoreItemCam1Select;
        using ItemCam1GetList = custom_interfaces::srv::DataStoreItemCam1GetList;

        using ItemCam2Insert = custom_interfaces::srv::DataStoreItemCam2Insert;
        using ItemCam2Delete = custom_interfaces::srv::DataStoreItemCam2Delete;
        using ItemCam2Select = custom_interfaces::srv::DataStoreItemCam2Select;
        using ItemCam2GetList = custom_interfaces::srv::DataStoreItemCam2GetList;
        DATA_STORE_PUBLIC
        explicit DataStore(const rclcpp::NodeOptions &options);
        ~DataStore();
        DATA_STORE_PUBLIC
        virtual void initNode() override;
        DATA_STORE_PUBLIC
        virtual void shutDownNode() override;

    private:
        helpers::Watchdog::SharedPtr _watchdog;
        DATA_STORE_LOCAL
        static void _signalHandler(int signum);

        rclcpp::Service<RgbDataInsert>::SharedPtr _rgb_data_insert_server;
        rclcpp::Service<RgbDataDelete>::SharedPtr _rgb_data_delete_server;
        rclcpp::Service<RgbDataSelect>::SharedPtr _rgb_data_select_server;
        rclcpp::Service<RgbDataGetList>::SharedPtr _rgb_data_get_list_server;

        rclcpp::Service<ItemCam1Insert>::SharedPtr _item_cam_1_insert_server;
        rclcpp::Service<ItemCam1Delete>::SharedPtr _item_cam_1_delete_server;
        rclcpp::Service<ItemCam1Select>::SharedPtr _item_cam_1_select_server;
        rclcpp::Service<ItemCam1GetList>::SharedPtr _item_cam_1_get_list_server;

        rclcpp::Service<ItemCam2Insert>::SharedPtr _item_cam_2_insert_server;
        rclcpp::Service<ItemCam2Delete>::SharedPtr _item_cam_2_delete_server;
        rclcpp::Service<ItemCam2Select>::SharedPtr _item_cam_2_select_server;
        rclcpp::Service<ItemCam2GetList>::SharedPtr _item_cam_2_get_list_server;

        rclcpp::Subscription<RgbDataSelect::Response>::SharedPtr _rgb_data_insert_sub;
        rclcpp::Subscription<ItemCam1Select::Response>::SharedPtr _item_cam_1_insert_sub;
        rclcpp::Subscription<ItemCam2Select::Response>::SharedPtr _item_cam_2_insert_sub;

        std::optional<RgbDataSelect::Response> _rgb_data_msg;
        std::optional<ItemCam1Select::Response> _item_cam1_msg;
        std::optional<ItemCam2Select::Response> _item_cam2_msg;

        DATA_STORE_LOCAL
        void _insertRgbData(const std::shared_ptr<RgbDataInsert::Request> request,
                            std::shared_ptr<RgbDataInsert::Response> response);
        DATA_STORE_LOCAL
        void _insertItemCam1(const std::shared_ptr<ItemCam1Insert::Request> request,
                             std::shared_ptr<ItemCam1Insert::Response> response);
        DATA_STORE_LOCAL
        void _insertItemCam2(const std::shared_ptr<ItemCam2Insert::Request> request,
                             std::shared_ptr<ItemCam2Insert::Response> response);
        DATA_STORE_LOCAL
        void _deleteRgbData(const std::shared_ptr<RgbDataDelete::Request> request,
                            std::shared_ptr<RgbDataDelete::Response> response);
        DATA_STORE_LOCAL
        void _deleteItemCam1(const std::shared_ptr<ItemCam1Delete::Request> request,
                             std::shared_ptr<ItemCam1Delete::Response> response);
        DATA_STORE_LOCAL
        void _deleteItemCam2(const std::shared_ptr<ItemCam2Delete::Request> request,
                             std::shared_ptr<ItemCam2Delete::Response> response);
        DATA_STORE_LOCAL
        void _selectRgbData(const std::shared_ptr<RgbDataSelect::Request> request,
                            std::shared_ptr<RgbDataSelect::Response> response);
        DATA_STORE_LOCAL
        void _selectItemCam1(const std::shared_ptr<ItemCam1Select::Request> request,
                             std::shared_ptr<ItemCam1Select::Response> response);
        DATA_STORE_LOCAL
        void _selectItemCam2(const std::shared_ptr<ItemCam2Select::Request> request,
                             std::shared_ptr<ItemCam2Select::Response> response);
        DATA_STORE_LOCAL
        void _getListRgbData(const std::shared_ptr<RgbDataGetList::Request> request,
                             std::shared_ptr<RgbDataGetList::Response> response);
        DATA_STORE_LOCAL
        void _getListItemCam1(const std::shared_ptr<ItemCam1GetList::Request> request,
                              std::shared_ptr<ItemCam1GetList::Response> response);
        DATA_STORE_LOCAL
        void _getListItemCam2(const std::shared_ptr<ItemCam2GetList::Request> request,
                              std::shared_ptr<ItemCam2GetList::Response> response);
        DATA_STORE_LOCAL
        DataState_e _validateInput();
        DATA_STORE_LOCAL
        DataState_e _validateOutput();

        std::unordered_map<double, RgbDataSelect::Response> _rgb_data;
        std::unordered_map<double, ItemCam1Select::Response> _item_cam1;
        std::unordered_map<double, ItemCam2Select::Response> _item_cam2;
    };
} // end of data_store
#endif
