#ifndef avena_view__AvenaView_H
#define avena_view__AvenaView_H

#include <rqt_gui_cpp/plugin.h>

#include <ui_avena_view.h>

// #include <opencv2/core/core.hpp>

#include <QAction>
#include <QImage>
#include <QList>
#include <QString>
#include <QSet>
#include <QSize>
#include <QWidget>
#include <QTimer>
#include <QResizeEvent>
#include <QProcess>
#include <QGraphicsScene>
#include <QTreeWidget>
#include <QMessageBox>
#include <QMetaType>
#include <vector>
#include <chrono>
#include <custom_interfaces/msg/heartbeat.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <custom_interfaces/msg/module_command.hpp>
#include <custom_interfaces/srv/control_command.hpp>
#include <custom_interfaces/action/bt_pick_and_place_action.hpp>
#include <custom_interfaces/action/simple_action.hpp>
#include <avena_view/config.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "rclcpp_action/rclcpp_action.hpp"
#include <rcl_interfaces/msg/log.hpp>
#include <custom_interfaces/srv/gui_pop_up.hpp>
#include <custom_interfaces/msg/gui_bt_message.hpp>
#include <QScrollBar>
#include <QProgressDialog>
#include <QDebug>
#include "utils.h"
#include "avena_view/detectron_runner.h"

Q_DECLARE_METATYPE(custom_interfaces::msg::GuiBtMessage::SharedPtr)
Q_DECLARE_METATYPE(custom_interfaces::msg::Heartbeat::SharedPtr)
Q_DECLARE_METATYPE(std_msgs::msg::String::SharedPtr)
Q_DECLARE_METATYPE(rcl_interfaces::msg::Log::SharedPtr)

using namespace std::chrono_literals;
namespace fs = std::filesystem;

using NodeListMap = std::map<std::string, custom_interfaces::msg::Heartbeat::SharedPtr>;
using NodeType = std::pair<std::string, custom_interfaces::msg::Heartbeat::SharedPtr>;
using PID = qint64;
using BTPickAndPlaceAction = custom_interfaces::action::BTPickAndPlaceAction;
using GoalHandleBTPickAndPlaceAction = rclcpp_action::ClientGoalHandle<BTPickAndPlaceAction>;
using CalibrateAction = custom_interfaces::action::SimpleAction;
using GoalCalibrateAction = rclcpp_action::ClientGoalHandle<CalibrateAction>;

namespace avena_view
{
    template <class T>
    static bool compareVectors(std::vector<T> a, std::vector<T> b)
    {
        if (a.size() != b.size())
        {
            return false;
        }
        ::std::sort(a.begin(), a.end());
        ::std::sort(b.begin(), b.end());
        return (a == b);
    }

    bool isItem(const std::string &path);
    bool isContrainer(const std::string &path);
    std::vector<std::string> getAreaNames(const std::string &file_path);

    enum class Status
    {
        STARTING,
        RUNNING,
        STOPPING,
        STOPPED,
        ERROR
    };

    class AvenaView : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT

    public:
        AvenaView();

        virtual void initPlugin(qt_gui_cpp::PluginContext &context);

        virtual void shutdownPlugin();

        virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;

        virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

    private slots:
        virtual void refreshNodeList();
        virtual void handleTableCellClick(int row, int col);
        virtual void runLaunchFile();
        virtual void terminateLaunchFile();
        virtual void startArmController();
        virtual void stopArmController();
        virtual void resumeArmController();
        virtual void pauseArmController();
        virtual void executeArmController();
        virtual void executePickPlace();
        virtual void cancelPickPlace();
        virtual void pausePickPlace();
        virtual void runCalibration();
        virtual void showQuestionMessageBox(custom_interfaces::msg::GuiBtMessage::SharedPtr msg);

        virtual void showSecurityRgbWarning();
        virtual void hideSecurityRgbWarning();

        virtual void publishDangerToolStatus();
        virtual void changeDangerToolStatus();
        virtual void publishSetBackgroundSignal();
        virtual void publishSceneChangeThreshold();
        virtual void publishPixelThreshold();

        virtual void refreshDangerToolStatus(bool msg);
        virtual void refreshSecurityPasueStatus(bool msg);
        virtual void refreshSecurityTriggerStatus(bool msg);

        virtual void refreshNodeList(custom_interfaces::msg::Heartbeat::SharedPtr msg);
        virtual void refreshLogConsole(std_msgs::msg::String::SharedPtr msg);
        virtual void refreshRosoutConsole(rcl_interfaces::msg::Log::SharedPtr msg);

        virtual void stopCalibrate();

    signals:
        void questionRecived(custom_interfaces::msg::GuiBtMessage::SharedPtr msg);
        void securityWarningRecived();
        void securityWarningClosed();
        void securityTriggerChanged(bool msg);
        void securityPauseChanged(bool msg);
        void dangerToolStatusChanged(bool msg);
        void nodeListChanged(custom_interfaces::msg::Heartbeat::SharedPtr msg);
        void logsAppeared(std_msgs::msg::String::SharedPtr msg);
        void rosOutAppeared(rcl_interfaces::msg::Log::SharedPtr msg);

    private:
        void heartBeatCallback(custom_interfaces::msg::Heartbeat::SharedPtr msg);
        void dangerToolStatusCallback(std_msgs::msg::Bool::SharedPtr msg);
        void btQuestionCallback(custom_interfaces::msg::GuiBtMessage::SharedPtr msg);
        void guiWarningCallback(std_msgs::msg::Bool::SharedPtr msg);
        void securityTriggerStatusCallback(std_msgs::msg::Bool::SharedPtr msg);
        void securityPauseStatusCallback(std_msgs::msg::Bool::SharedPtr msg);

        void runCalibrationLaunchFile();

        bool killAllChildProcessPids(PID launch_file_pid);

        void subscribeHeartBeat();
        void subscribeLogs();
        void subscribeRosOut();
        void subscribeBtQuestion();

        void addNodeToList(const std::string &node_name);
        void setUpNodesTabContent(const NodeType &node, int row);
        void sendArmCommand(ControlCommands command);
        void writeLog(QString msg, QTextBrowser *dst_ptr);
        void setIndicator(const QString status_msg, const QColor color, QGraphicsScene *led, QLabel *label);
        void logsCallback(std_msgs::msg::String::SharedPtr msg);
        void rosoutCallback(rcl_interfaces::msg::Log::SharedPtr msg);
        void setUpPickPlaceComboBoxes();
        void setUpStoppedPickPlaceUi();
        void setUpStartedPickPlaceUi();
        void fillNodesList();
        void setUpRunningArmControlUi();
        void writeTerminalAndUiLog(const char *msg, Status status, QTextBrowser *console);
        void fillConfigTree();
        void startNodes();
        void setUpDangerToolTimer();

        void setUpIdBasedOnSavedPid();

        void setUpGuiWarningUi();

        void sendPickPlaceGoal(const std::string &command);
        void pickPlaceGoalResponseCallback(std::shared_future<GoalHandleBTPickAndPlaceAction::SharedPtr> future);
        void pickPlaceFeedbackCallback(
            GoalHandleBTPickAndPlaceAction::SharedPtr,
            const std::shared_ptr<const BTPickAndPlaceAction::Feedback> feedback);
        void pickPlaceResultCallback(const GoalHandleBTPickAndPlaceAction::WrappedResult &result);

        void calibrateGoalResponseCallback(std::shared_future<GoalCalibrateAction::SharedPtr> future);
        void calibrateFeedbackCallback(
            GoalCalibrateAction::SharedPtr,
            const std::shared_ptr<const CalibrateAction::Feedback> feedback);
        void calibrateResultCallback(const GoalCalibrateAction::WrappedResult &result);

        void runGuiPopUpTest(const std::shared_ptr<custom_interfaces::srv::GUIPopUp::Request> request, std::shared_ptr<custom_interfaces::srv::GUIPopUp::Response> response);

        std::chrono::nanoseconds rosTime2Chrono(builtin_interfaces::msg::Time &stamp);

        Ui::AvenaViewWidget ui_;
        QWidget *widget_;
        std::shared_ptr<QTimer> refreshing_node_list_timer_;
        std::shared_ptr<QTimer> publishing_danger_tool_status_timer_;
        std::shared_ptr<QMessageBox> security_rgb_warning_;

        std::shared_ptr<QGraphicsScene> arm_control_graphics_scene_;
        std::shared_ptr<QGraphicsScene> pick_place_graphics_scene_;

        // QList<QTreeWidgetItem*> parameters;

        QProcess *launch_file_process_;
        qint64 launch_file_pid_;
        std::string pid_file_name_;

        QProcess *calibrate_launch_file_process_;
        qint64 calibrate_launch_file_pid_;

        // QScrollBar *sb_;

        NodeListMap node_list_;
        bool refreshing_on_;
        bool danger_tool_status_;
        bool previus_gui_warning_msg_;

        Status pick_place_status_;

        rclcpp::Subscription<custom_interfaces::msg::Heartbeat>::SharedPtr heartbeat_checker_;
        rclcpp::Publisher<custom_interfaces::msg::ModuleCommand>::SharedPtr command_publisher_;
        rclcpp::Client<custom_interfaces::srv::ControlCommand>::SharedPtr arm_command_client_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr logs_sub_;
        rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
        rclcpp_action::Client<BTPickAndPlaceAction>::SharedPtr pick_place_action_client_;
        rclcpp_action::Client<custom_interfaces::action::SimpleAction>::SharedPtr calibrate_action_client_;

        rclcpp::Publisher<custom_interfaces::msg::GuiBtMessage>::SharedPtr user_answer_pub_;
        rclcpp::Subscription<custom_interfaces::msg::GuiBtMessage>::SharedPtr bt_question_sub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr danger_tool_in_hand_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr scene_change_treshold_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pixel_treshold_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr set_background_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gui_warning_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr security_trigger_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr security_pause_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr danger_tool_in_hand_sub_;

        std::shared_ptr<DetectronRunner> detectron_runner_;
    };
}

#endif