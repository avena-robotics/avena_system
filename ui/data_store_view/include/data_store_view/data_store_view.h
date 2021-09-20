#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_data_store_view.h>
#include <data_store_view/cameras_view.h>
#include <data_store_view/items_view.h>

namespace data_store_view
{
    class DataStoreView : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        DataStoreView();
        ~DataStoreView();

        virtual void initPlugin(qt_gui_cpp::PluginContext &context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

    private:
        Ui::DataStoreView _ui;
        QWidget *_widget;

        size_t _cameras_amount;
    
        std::shared_ptr<CamerasView> _cameras_view;
        std::shared_ptr<ItemsView> _items_view;
        std::map<std::string, size_t> debug_map;
    private slots:
        void getAmountOfCameras();
    };
}