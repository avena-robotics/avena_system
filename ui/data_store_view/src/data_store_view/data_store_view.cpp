#include <data_store_view/data_store_view.h>
#include <pluginlib/class_list_macros.hpp>

namespace data_store_view
{
    DataStoreView::DataStoreView()
        : rqt_gui_cpp::Plugin(), _widget(0)
    {
        debug();
    }

    DataStoreView::~DataStoreView()
    {
    }

    void DataStoreView::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        debug();
        _widget = new QWidget();
        _ui.setupUi(_widget);
        if (context.serialNumber() > 1)
        {
            _widget->setWindowTitle(_widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(_widget);
        connect(_ui.refreshButton, SIGNAL(clicked(bool)), this, SLOT(getAmountOfCameras()));

        debug();
        getAmountOfCameras();
        debug();
        if(_cameras_amount < 1)
            throw error_str("Amount of cameras is invalid, make sure parameters server is up and running");
        debug();
        _cameras_view = std::make_shared<CamerasView>(&_ui, node_, _cameras_amount);
        debug();
        _items_view = std::make_shared<ItemsView>(&_ui, node_, _widget, _cameras_amount);
        debug();

        try{
            debug();
            _cameras_view->setUp();
            debug();
            _items_view->setUp();
            debug();
        }
        catch(std::runtime_error& e)
        {
            std::cout << e.what() << std::endl;
        }
    }

    void DataStoreView::shutdownPlugin()
    {
        _cameras_view = nullptr;
        _items_view = nullptr;
    }

    void DataStoreView::getAmountOfCameras()
    {
        std::cout << "Reading parameters from the server" << std::endl;

        auto parameters = helpers::commons::getParameters({"cameras"});
        if (parameters.empty())
        {
            std::cout << "Can't read parameters from server..." << std::endl;
        }
        else
        {
            _cameras_amount = parameters["cameras"]["cameras_amount"];
            std::cout << "Parameters read successfully..." << std::endl;
            std::cout << "Amount of cameras: " << _cameras_amount << std::endl;
        }
    }

    void DataStoreView::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
    {
    }
    
    void DataStoreView::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
    {
    }
}

PLUGINLIB_EXPORT_CLASS(data_store_view::DataStoreView, rqt_gui_cpp::Plugin)