#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <ui_robot_diagnostics.h>


namespace robot_diagnostics {
    class RobotDiagnostics : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT
    public:
        RobotDiagnostics();
        ~RobotDiagnostics();

        virtual void initPlugin(qt_gui_cpp::PluginContext &context);
        virtual void shutdownPlugin();
        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

    private:
        Ui::RobotDiagnostics _ui;
        QWidget *_widget;
    };
}