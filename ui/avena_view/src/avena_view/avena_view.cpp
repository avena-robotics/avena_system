#include <avena_view/avena_view.h>
#include <pluginlib/class_list_macros.hpp>

namespace avena_view
{
    AvenaView::AvenaView()
        : rqt_gui_cpp::Plugin(), _widget(0)
    {
        setObjectName("AvenaView");
    }

    AvenaView::~AvenaView()
    {
    }

    void AvenaView::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        _widget = new QWidget();
        _ui.setupUi(_widget);

        if (context.serialNumber() > 1)
        {
            _widget->setWindowTitle(_widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }

        context.addWidget(_widget);

        _nodes_list = std::make_shared<NodesList>(&_ui, node_);
        _start_system = std::make_shared<StartSystem>(&_ui, node_, _nodes_list->nodes_map);
        _fast_loop = std::make_shared<FastLoop>(&_ui, node_, _widget);
        _robot_control = std::make_shared<RobotControl>(&_ui, node_);
        _vision_actions = std::make_shared<VisionActions>(&_ui, node_);
    }

    void AvenaView::shutdownPlugin(){
        _nodes_list = nullptr;
        _start_system = nullptr;
        _fast_loop = nullptr;
        _robot_control = nullptr;
        _vision_actions = nullptr;
    }

    void AvenaView::saveSettings([[maybe_unused]] qt_gui_cpp::Settings &plugin_settings, [[maybe_unused]] qt_gui_cpp::Settings &instance_settings) const {}

    void AvenaView::restoreSettings([[maybe_unused]] const qt_gui_cpp::Settings &plugin_settings,[[maybe_unused]] const qt_gui_cpp::Settings &instance_settings) {}

}

PLUGINLIB_EXPORT_CLASS(avena_view::AvenaView, rqt_gui_cpp::Plugin)