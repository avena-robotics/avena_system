#include <robot_diagnostics/robot_diagnostics.h>
#include <pluginlib/class_list_macros.hpp>

namespace robot_diagnostics
{
    RobotDiagnostics::RobotDiagnostics()
        : rqt_gui_cpp::Plugin(), _widget(0)
    {
    }
    RobotDiagnostics::~RobotDiagnostics()
    {
    }

    void RobotDiagnostics::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        _widget = new QWidget();
        _ui.setupUi(_widget);
        if(context.serialNumber() > 1)
        {
            _widget->setWindowTitle(_widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }

        context.addWidget(_widget);
    }


    void RobotDiagnostics::shutdownPlugin()
    {
        _widget = nullptr;
    }
    void RobotDiagnostics::saveSettings([[maybe_unused]] qt_gui_cpp::Settings &plugin_settings, [[maybe_unused]] qt_gui_cpp::Settings &instance_settings) const
    {
    }
    void RobotDiagnostics::restoreSettings([[maybe_unused]] const qt_gui_cpp::Settings &plugin_settings, [[maybe_unused]] const qt_gui_cpp::Settings &instance_settings)
    {
    }
} // namespace robot_diagnostics

PLUGINLIB_EXPORT_CLASS(robot_diagnostics::RobotDiagnostics, rqt_gui_cpp::Plugin)
