#ifndef _DATA_STORE_HPP
#define _DATA_STORE_HPP

#include "helpers_commons/helpers_commons.hpp"
#include "rclcpp/rclcpp.hpp"
#include <csignal>
#include "visibility_control.h"
// ___Avena___
#include "data_types.hpp"
namespace data_store
{
    class DataStore : public rclcpp::Node, public helpers::WatchdogInterface
    {
    public:
        DATA_STORE_PUBLIC
        explicit DataStore(const rclcpp::NodeOptions &options);
        ~DataStore();
        DATA_STORE_PUBLIC
        virtual void initNode() override;
        DATA_STORE_PUBLIC
        virtual void shutDownNode() override;

    private:
        helpers::Watchdog::SharedPtr _watchdog;
        std::unique_ptr<RgbdSync> _cameras_data_element_ptr;
        std::unique_ptr<Tracker> _tracker_element_ptr;
        std::unique_ptr<Detectron> _detectron_element_ptr;
        std::unique_ptr<Items> _items_element_ptr;
        std::unique_ptr<Scene> _scene_element_ptr;
        // add other data elements here
    };

} // end of data_store

#endif
