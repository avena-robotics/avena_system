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
        std::unique_ptr<RgbData> _rgb_data_element_ptr;
        std::unique_ptr<ItemCam1> _item_cam1_element_ptr;
        std::unique_ptr<ItemCam2> _item_cam2_element_ptr;
        // add other data elements here
    };

} // end of data_store

#endif
