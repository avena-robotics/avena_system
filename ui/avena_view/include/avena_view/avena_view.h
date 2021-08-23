#ifndef avena_view__AvenaView_H
#define avena_view__AvenaView_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_avena_view.h>

#include <avena_view/nodes_list.h>
#include <avena_view/start_system.h>
#include <avena_view/fast_loop.h>
#include <avena_view/robot_control.h>


namespace avena_view
{

    class AvenaView : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT

    public:
        AvenaView();
        ~AvenaView();

        virtual void initPlugin(qt_gui_cpp::PluginContext &context);

        virtual void shutdownPlugin();

        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;

        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

        Ui::AvenaViewWidget _ui;
        QWidget *_widget;

        std::shared_ptr<NodesList> _nodes_list;
        std::shared_ptr<StartSystem> _start_system;
        std::shared_ptr<FastLoop> _fast_loop;
        std::shared_ptr<RobotControl> _robot_control;
    };
}

#endif