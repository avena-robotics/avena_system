#include <avena_view/avena_view.h>
#include <pluginlib/class_list_macros.hpp>

namespace avena_view
{
    AvenaView::AvenaView()
        : rqt_gui_cpp::Plugin(), widget_(0)
    {
        setObjectName("AvenaView");
    }

#pragma region QT_PLUGIN

    void AvenaView::initPlugin(qt_gui_cpp::PluginContext &context)
    {
        // signal(SIGINT, std::bind(&AvenaView::sigIntHandle, this));

        // QApplication::setAttribute(Qt::AA_EnableHighDpiScaling); // DPI support
        // QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
        widget_ = new QWidget();
        ui_.setupUi(widget_);

        if (context.serialNumber() > 1)
        {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);

        qRegisterMetaType<custom_interfaces::msg::GuiBtMessage::SharedPtr>("custom_interfaces::msg::GuiBtMessage::SharedPtr");
        qRegisterMetaType<custom_interfaces::msg::Heartbeat::SharedPtr>("custom_interfaces::msg::Heartbeat::SharedPtr");
        qRegisterMetaType<std_msgs::msg::String::SharedPtr>("std_msgs::msg::String::SharedPtr");
        qRegisterMetaType<rcl_interfaces::msg::Log::SharedPtr>("rcl_interfaces::msg::Log::SharedPtr");

        // fillNodesList();

        arm_control_graphics_scene_ = std::make_shared<QGraphicsScene>(widget_);
        pick_place_graphics_scene_ = std::make_shared<QGraphicsScene>(widget_);
        ui_.statusIndicatorArmControl->setScene(arm_control_graphics_scene_.get());
        ui_.statusIndicator->setScene(pick_place_graphics_scene_.get());

        launch_file_process_ = new QProcess();
        calibrate_launch_file_process_ = new QProcess();

        calibrate_launch_file_pid_ = 0;

        refreshing_node_list_timer_ = std::make_shared<QTimer>(this);
        refreshing_on_ = true;
        connect(refreshing_node_list_timer_.get(), SIGNAL(timeout()), this, SLOT(refreshNodeList()));
        refreshing_node_list_timer_->start(GUI_REFRESH_DELAY);

        connect(ui_.nodeListTable, SIGNAL(cellClicked(int, int)), this, SLOT(handleTableCellClick(int, int)));

        connect(ui_.startButtonArmControl, SIGNAL(clicked(bool)), this, SLOT(startArmController()));
        connect(ui_.stopButtonArmControl, SIGNAL(clicked(bool)), this, SLOT(stopArmController()));
        connect(ui_.resumeButtonArmControl, SIGNAL(clicked(bool)), this, SLOT(resumeArmController()));
        connect(ui_.pauseButtonArmControl, SIGNAL(clicked(bool)), this, SLOT(pauseArmController()));

        connect(ui_.startButton, SIGNAL(clicked(bool)), this, SLOT(runLaunchFile()));
        connect(ui_.stopButton, SIGNAL(clicked(bool)), this, SLOT(terminateLaunchFile()));

        connect(ui_.executeButton, SIGNAL(clicked(bool)), this, SLOT(executePickPlace()));
        connect(ui_.resumeButton, SIGNAL(clicked(bool)), this, SLOT(executePickPlace()));
        connect(ui_.pauseButton, SIGNAL(clicked(bool)), this, SLOT(pausePickPlace()));
        connect(ui_.calibrateButton, SIGNAL(clicked(bool)), this, SLOT(runCalibration()));

        //TODO: put all subs in one method
        QTimer::singleShot(100, this, &AvenaView::subscribeHeartBeat);
        QTimer::singleShot(100, this, &AvenaView::subscribeLogs);
        QTimer::singleShot(100, this, &AvenaView::subscribeRosOut);
        QTimer::singleShot(100, this, &AvenaView::subscribeBtQuestion);

        // sb_ = ui_.dangerToolConsole->verticalScrollBar();
        danger_tool_in_hand_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/danger_tool_in_hand",
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&AvenaView::dangerToolStatusCallback, this, std::placeholders::_1));

        gui_warning_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/gui_warning",
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&AvenaView::guiWarningCallback, this, std::placeholders::_1));

        security_trigger_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/security_trigger",
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&AvenaView::securityTriggerStatusCallback, this, std::placeholders::_1));

        security_pause_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/security_pause",
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&AvenaView::securityPauseStatusCallback, this, std::placeholders::_1));

        command_publisher_ = node_->create_publisher<custom_interfaces::msg::ModuleCommand>("/system_monitor/command", 10);
        user_answer_pub_ = node_->create_publisher<custom_interfaces::msg::GuiBtMessage>("/user_answer", 10);
        arm_command_client_ = node_->create_client<custom_interfaces::srv::ControlCommand>("arm_controller/commands");
        pick_place_action_client_ = rclcpp_action::create_client<BTPickAndPlaceAction>(node_, PICK_PLACE_ACTION_SERVER_NAME);
        calibrate_action_client_ = rclcpp_action::create_client<custom_interfaces::action::SimpleAction>(node_, CALIBRATE_ACTION_SERVER_NAME);

        danger_tool_in_hand_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/danger_tool_in_hand", 10);
        set_background_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/rgbdiff_set_background", 10);
        scene_change_treshold_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/rgbdiff_scene_change_threshold", 10);
        pixel_treshold_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/rgbdiff_pixel_threshold_threshold", 10);

        pid_file_name_ = ament_index_cpp::get_package_share_directory("avena_view") + "/" + PID_FILE_NAME;

        setUpDangerToolTimer();
        connect(ui_.dangerToolSwitch, SIGNAL(clicked(bool)), this, SLOT(changeDangerToolStatus()));

        setUpPickPlaceComboBoxes();

        setUpIdBasedOnSavedPid();

        connect(this, SIGNAL(questionRecived(custom_interfaces::msg::GuiBtMessage::SharedPtr)), this, SLOT(showQuestionMessageBox(custom_interfaces::msg::GuiBtMessage::SharedPtr)));
        connect(this, SIGNAL(securityWarningRecived()), this, SLOT(showSecurityRgbWarning()));

        connect(ui_.setBackgroundButton, SIGNAL(clicked()), SLOT(publishSetBackgroundSignal()));
        connect(ui_.setSceneChangeTresholdButton, SIGNAL(clicked()), SLOT(publishSceneChangeThreshold()));
        connect(ui_.setPerPixelTresholdButton, SIGNAL(clicked()), this, SLOT(publishPixelThreshold()));

        connect(this, SIGNAL(dangerToolStatusChanged(bool)), this, SLOT(refreshDangerToolStatus(bool)));
        connect(this, SIGNAL(securityTriggerChanged(bool)), this, SLOT(refreshSecurityTriggerStatus(bool)));
        connect(this, SIGNAL(securityPauseChanged(bool)), this, SLOT(refreshSecurityPasueStatus(bool)));

        connect(this, SIGNAL(nodeListChanged(custom_interfaces::msg::Heartbeat::SharedPtr)), this, SLOT(refreshNodeList(custom_interfaces::msg::Heartbeat::SharedPtr)));

        connect(this, SIGNAL(logsAppeared(std_msgs::msg::String::SharedPtr)), this, SLOT(refreshLogConsole(std_msgs::msg::String::SharedPtr)));
        connect(this, SIGNAL(rosOutAppeared(rcl_interfaces::msg::Log::SharedPtr)), this, SLOT(refreshRosoutConsole(rcl_interfaces::msg::Log::SharedPtr)));

        previus_gui_warning_msg_ = false;
        setUpGuiWarningUi();
        connect(this, SIGNAL(securityWarningClosed()), this, SLOT(hideSecurityRgbWarning()));

        detectron_runner_ = std::make_shared<DetectronRunner>(&ui_);

        connect(ui_.calibrateCancelButton, SIGNAL(clicked(bool)), this, SLOT(stopCalibrate()));

        cameras_launch_file_ = std::make_shared<LaunchFile>("cameras.pid");
        connect(ui_.startCamerasButton, SIGNAL(clicked(bool)), this, SLOT(startCameras()));
        connect(ui_.stopCamerasButton, SIGNAL(clicked(bool)), this, SLOT(stopCameras()));
        connect(ui_.res1080Radio, SIGNAL(clicked()), this, SLOT(switchResolutions()));
        connect(ui_.res720Radio, SIGNAL(clicked()), this, SLOT(switchResolutions()));
        ui_.res720Radio->setChecked(false);
        ui_.res1080Radio->setChecked(true);
    }

    void AvenaView::switchResolutions()
    {
        if (ui_.res720Radio->isChecked())
        {
            ui_.res1080Radio->setChecked(false);
            ui_.res720Radio->setChecked(true);
        }
        else
        {
            ui_.res720Radio->setChecked(false);
            ui_.res1080Radio->setChecked(true);
        }

        ui_.logConsoleCameras->append(
            (ui_.res720Radio->isChecked())?"720p ON":"720p OFF"
        );

        ui_.logConsoleCameras->append(
            (ui_.res1080Radio->isChecked())?"1080p ON":"1080p OFF"
        );
    }

    void AvenaView::startCameras()
    {
        bool full_hd = ui_.res1080Radio->isChecked();
        ui_.logConsoleCameras->append( (full_hd) ? "1080p mode" : "720p mode");
        ui_.startCamerasButton->setEnabled(false);
        ui_.stopCamerasButton->setEnabled(false);

        if (full_hd)
        {
            cameras_launch_file_->setArguments({"launch",
                                                "realsense2_camera",
                                                "rs_multi_camera_launch.py",
                                                "rgb_width1:=1920",
                                                "rgb_height1:=1080",
                                                "rgb_width2:=1920",
                                                "rgb_height2:=1080"});
        }
        else
        {
            cameras_launch_file_->setArguments({"launch",
                                                "realsense2_camera",
                                                "rs_multi_camera_launch.py",
                                                "rgb_width1:=1280",
                                                "rgb_height1:=720",
                                                "rgb_width2:=1280",
                                                "rgb_height2:=720"});
        }
        auto post_cameras_start = [this]()
        {
            ui_.startCamerasButton->setEnabled(false);
            ui_.stopCamerasButton->setEnabled(true);
            ui_.logConsoleCameras->append("Started cameras");
        };
        cameras_launch_file_->run(post_cameras_start, 5000);
    }

    void AvenaView::stopCameras()
    {
        ui_.logConsoleCameras->append("Stoping cameras");
        cameras_launch_file_->terminate();
        ui_.startCamerasButton->setEnabled(true);
        ui_.stopCamerasButton->setEnabled(false);
    }

    void AvenaView::setUpIdBasedOnSavedPid()
    {
        std::ifstream pid_file_in(pid_file_name_);
        if (!pid_file_in)
        {
            setUpStoppedPickPlaceUi();
            return;
        }

        int saved_pid = 0;
        pid_file_in >> saved_pid;

        if (saved_pid != 0)
        {
            writeLog("Restoring started pick and place session", ui_.logConsole);
            launch_file_pid_ = saved_pid;
            setUpStartedPickPlaceUi();
        }
        pid_file_in.close();

        std::ofstream pid_file_out;
        pid_file_out.open(pid_file_name_, std::ofstream::out | std::ofstream::trunc);

        pid_file_out.close();
    }

    void AvenaView::changeDangerToolStatus()
    {
        danger_tool_status_ = !danger_tool_status_;
        if (danger_tool_status_)
        {
            ui_.dangerToolSwitch->setText(tr("DANGER_TOOL: ON"));
        }
        else
        {
            ui_.dangerToolSwitch->setText(tr("DANGER_TOOL: OFF"));
        }
    }

    void AvenaView::setUpDangerToolTimer()
    {
        ui_.dangerToolSwitch->setText("DANGER_TOOL: OFF");
        danger_tool_status_ = false;
        publishing_danger_tool_status_timer_ = std::make_shared<QTimer>(this);
        refreshing_on_ = true;
        connect(publishing_danger_tool_status_timer_.get(), SIGNAL(timeout()), this, SLOT(publishDangerToolStatus()));
        publishing_danger_tool_status_timer_->start(GUI_REFRESH_DELAY);
    }

    void AvenaView::publishDangerToolStatus()
    {
        std_msgs::msg::Bool msg;
        msg.data = danger_tool_status_;
        danger_tool_in_hand_pub_->publish(msg);
    }

    void AvenaView::shutdownPlugin()
    {
        if (pick_place_status_ == Status::RUNNING)
            terminateLaunchFile();

        if (cameras_launch_file_->isRunning())
            cameras_launch_file_->terminate();

        stopCalibrate();
        refreshing_on_ = false;
        refreshing_node_list_timer_->stop();
    }

    void AvenaView::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const {}

    void AvenaView::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings) {}

#pragma endregion

#pragma region UTILS

    void AvenaView::sigIntHandle(int signum)
    {
        shutdownPlugin();
        exit(signum);
    }

    void AvenaView::setUpGuiWarningUi()
    {
        security_rgb_warning_ = std::make_shared<QMessageBox>();
        security_rgb_warning_->setText("Security Area triggered. Waiting...");
    }

    void AvenaView::writeTerminalAndUiLog(const char *msg, Status status, QTextBrowser *console)
    {
        RCLCPP_INFO(node_->get_logger(), msg);
        writeLog(msg, console);
        switch (status)
        {
        case Status::RUNNING:
            setIndicator("READY", Qt::green, pick_place_graphics_scene_.get(), ui_.statusLabel);
            break;
        case Status::STOPPED:
            setIndicator("STOPPED", Qt::red, pick_place_graphics_scene_.get(), ui_.statusLabel);
            break;
        case Status::ERROR:
            setIndicator("ERROR", Qt::darkRed, pick_place_graphics_scene_.get(), ui_.statusLabel);
            break;
        default:
            setIndicator("NOT READY", Qt::yellow, pick_place_graphics_scene_.get(), ui_.statusLabel);
        }
    }

    bool isItem(const std::string &path)
    {
        auto file = YAML::LoadFile(path);
        if (file["item"])
        {
            return file["item"].as<bool>();
        }
        return false;
    }

    bool isContrainer(const std::string &path)
    {
        auto file = YAML::LoadFile(path);
        if (file["is_container"])
        {
            return file["is_container"].as<bool>();
        }
        return false;
    }

    std::chrono::nanoseconds AvenaView::rosTime2Chrono(builtin_interfaces::msg::Time &stamp)
    {
        auto seconds = std::chrono::seconds(stamp.sec);
        auto nanoseconds = std::chrono::nanoseconds(stamp.nanosec);

        return seconds + nanoseconds;
    }

    void AvenaView::writeLog(QString msg, QTextBrowser *dst_ptr)
    {
        dst_ptr->append(QString("$> ") + msg);
    }

    void AvenaView::setIndicator(const QString status_msg, const QColor color, QGraphicsScene *led, QLabel *label)
    {
        label->setText(status_msg);
        QBrush led_brush = QBrush(color);
        led->addEllipse(0, 0, 45, 45, QPen(color), led_brush);
    }

    std::vector<std::string> getAreaNames(const std::string &file_path)
    {
        std::ifstream in_file(file_path);
        std::vector<std::string> result;

        std::string line;

        while (in_file >> line)
        {
            if (line.rfind("area") != std::string::npos)
            {
                line.erase(line.end() - 1, line.end());
                result.push_back(line);
            }
        }

        in_file.close();
        return result;
    }

#pragma endregion

#pragma region NODES_LIFE_CYCLE
    void AvenaView::startNodes()
    {
        std::for_each(
            node_list_.begin(),
            node_list_.end(),
            [this](NodeType node)
            {
                auto module_command = std::make_shared<custom_interfaces::msg::ModuleCommand>();
                module_command->command = custom_interfaces::msg::ModuleCommand::START;
                module_command->header = std_msgs::msg::Header();
                module_command->header.stamp = this->node_->now();
                module_command->module_name = node.first;
                this->command_publisher_->publish(*module_command);
            });
        QTimer::singleShot(500, this, &AvenaView::publishSetBackgroundSignal);
        widget_->update();
    }
    void AvenaView::addNodeToList(const std::string &node_name)
    {
        node_list_.insert({node_name, nullptr});
    }

    void AvenaView::fillNodesList()
    {
        std::string parameters_package_path = ament_index_cpp::get_package_share_directory("parameters_server");
        std::string node_list_path = parameters_package_path + "/config/parameters/globals/watched_nodes.yaml";
        try
        {
            auto nodes_yaml = YAML::LoadFile(node_list_path);
            if (nodes_yaml["nodes"])
            {
                auto nodes_names_list = nodes_yaml["nodes"].as<std::vector<std::string>>();
                std::for_each(
                    nodes_names_list.begin(),
                    nodes_names_list.end(),
                    [this](const std::string &node_name)
                    {
                        this->addNodeToList(node_name);
                    });
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(node_->get_logger(), "Failed to load nodes list: %s", e.what());
        }
    }

    void AvenaView::setUpNodesTabContent(const NodeType &node, int row)
    {
        int heartbeat_delay = 0;

        QString status_string;
        QColor status_color;
        QString since_last;
        QString command_button_text;
        QColor command_button_color;

        if (node.second != nullptr)
        {
            auto now = std::chrono::system_clock::now();
            auto heartbeat_time = rosTime2Chrono(node.second->header.stamp);

            heartbeat_delay = (now - heartbeat_time).time_since_epoch().count() / MILLI_NANO_MULTIPLIER;
            since_last = QString::number(heartbeat_delay) + " [ms]";
        }

        if (node.second == nullptr || heartbeat_delay > MAX_HEARTBEAT_DELAY)
        {
            status_string = "TERMINATED";
            status_color = Qt::darkRed;
            since_last = "N/A";
            command_button_text = "N/A";
            command_button_color = Qt::gray;
        }
        else
        {
            switch (node.second->status)
            {
            case custom_interfaces::msg::Heartbeat::STARTING:
                status_string = "STARTING";
                status_color = Qt::yellow;
                command_button_text = "N/A";
                command_button_color = Qt::gray;
                break;
            case custom_interfaces::msg::Heartbeat::RUNNING:
                status_string = "RUNNING";
                status_color = Qt::green;
                command_button_text = "STOP";
                command_button_color = Qt::red;
                break;
            case custom_interfaces::msg::Heartbeat::STOPPING:
                status_string = "STOPPING";
                status_color = Qt::yellow;
                command_button_text = "N/A";
                command_button_color = Qt::gray;
                break;
            case custom_interfaces::msg::Heartbeat::STOPPED:
                status_string = "STOPPED";
                status_color = Qt::red;
                command_button_text = "START";
                command_button_color = Qt::darkGreen;
                break;
            }
        }

        auto q_name = new QTableWidgetItem(QString(node.first.c_str()), Qt::DisplayRole);
        auto q_status = new QTableWidgetItem(status_string, Qt::DisplayRole);
        auto q_since_last = new QTableWidgetItem(since_last, Qt::DisplayRole);
        auto q_command_button = new QTableWidgetItem(command_button_text, Qt::DisplayRole);

        q_status->setBackgroundColor(status_color);
        q_command_button->setBackgroundColor(command_button_color);
        q_command_button->setSelected(false);

        ui_.nodeListTable->setItem(row, 0, q_name);
        ui_.nodeListTable->setItem(row, 1, q_since_last);
        ui_.nodeListTable->setItem(row, 2, q_status);
        ui_.nodeListTable->setItem(row, 3, q_command_button);
    }

    void AvenaView::refreshNodeList()
    {
        if (refreshing_on_)
        {
            // RCLCPP_INFO_STREAM(node_->get_logger(), node_list_.size());
            ui_.nodeListTable->setRowCount(node_list_.size());
            int row = 0;
            for (const auto &node : node_list_)
            {
                setUpNodesTabContent(node, row++);
            }
        }
    }

    void AvenaView::handleTableCellClick(int row, int col)
    {
        if (col != COMMAND_COL)
            return;

        std::string command = ui_.nodeListTable->item(row, COMMAND_COL)->text().toStdString();
        std::string module_name = ui_.nodeListTable->item(row, NODE_NAME_COL)->text().toStdString();
        auto module_command = std::make_shared<custom_interfaces::msg::ModuleCommand>();

        if (command == START_ACTION_COMMAND)
        {
            module_command->command = custom_interfaces::msg::ModuleCommand::START;
        }
        else if (command == STOP_ACTION_COMMAND)
        {
            module_command->command = custom_interfaces::msg::ModuleCommand::STOP;
        }
        else
        {
            std::cout << "Unrecognized command. Skipping." << std::endl;
            return;
        }
        module_command->header = std_msgs::msg::Header();
        module_command->header.stamp = node_->now();
        module_command->module_name = module_name;

        command_publisher_->publish(*module_command);

        std::cout << "Sent command: " << command << " to: " << module_name << " module" << std::endl;
    }

#pragma endregion

#pragma region SYSTEM_START_STOP

    void AvenaView::runLaunchFile()
    {
        RCLCPP_INFO(node_->get_logger(), "Starting system");
        QString program = "ros2";

        launch_file_process_->setArguments({"launch", "avena_bringup", START_LAUNCH_FILE});
        launch_file_process_->setProgram(program);
        launch_file_pid_ = 0;

        std::ofstream pid_file(pid_file_name_);

        if (!pid_file)
        {
            throw std::runtime_error(std::string("Cannot create pid file. Exiting"));
        }

        if (launch_file_process_->startDetached(&launch_file_pid_))
        {
            this->writeTerminalAndUiLog("Starting pick place system", Status::STARTING, ui_.logConsole);
            pid_file << launch_file_pid_;
            auto post_start_action = [this]()
            {
                this->startNodes();
                this->writeTerminalAndUiLog("Sucessfully started pick place system", Status::RUNNING, ui_.logConsole);
                this->setUpStartedPickPlaceUi();
                // QTimer::singleShot(100, this, [this]()
                //                    { sendPickPlaceGoal("start"); });
            };
            QTimer::singleShot(3000, this, post_start_action);
        }
        else
        {
            writeTerminalAndUiLog("Error while starting pick place system", Status::ERROR, ui_.logConsole);
        }

        pid_file.close();

        // writeLog("PID=" + QString::number(launch_file_pid_), ui_.logConsole);
    }

    void AvenaView::terminateLaunchFile()
    {
        RCLCPP_INFO(node_->get_logger(), "Stoping system");

        if (launch_file_pid_ > 0)
        {
            //TODO: change returend value type to int with exit code
            if (killAllChildProcessPids(launch_file_pid_))
            {
                writeTerminalAndUiLog("Sucessfully stopped pick place system", Status::STOPPED, ui_.logConsole);
                fs::remove(pid_file_name_);
                setUpStoppedPickPlaceUi();
            }
            else
            {
                writeTerminalAndUiLog("Error while stopping pick place system", Status::ERROR, ui_.logConsole);
            }
        }
        else
        {
            writeTerminalAndUiLog("Nothing to stop", Status::ERROR, ui_.logConsole);
        }
    }

#pragma endregion

#pragma region ARM_CONTROL

    void AvenaView::sendArmCommand(ControlCommands command)
    {
        int waiting_counter = 0;
        auto request = std::make_shared<custom_interfaces::srv::ControlCommand::Request>();
        request->command = static_cast<int>(command);
        while (waiting_counter++ < 3 && !arm_command_client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {

                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                writeLog("Interrupted while waiting for the service. Exiting.", ui_.logConsoleArmControl);
                return;
            }

            setIndicator("WAITING", Qt::yellow, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            writeLog("service not available, waiting again...", ui_.logConsoleArmControl);
        }

        if (waiting_counter < 3)
        {
            auto result = arm_command_client_->async_send_request(request);
            std::future_status status = result.wait_for(std::chrono::seconds(1));
            if (status == std::future_status::ready)
            {
                RCLCPP_INFO_STREAM(node_->get_logger(), "ArmController response: " << result.get()->error);
                writeLog("ArmController response: " + QString(result.get()->error.c_str()), ui_.logConsoleArmControl);
                setIndicator("SUCCESS", Qt::green, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
            }
            else
            {
                setIndicator("ERROR", Qt::darkRed, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service ");
                writeLog("Failed to call service", ui_.logConsoleArmControl);
            }
        }
        else
        {
            setIndicator("ERROR", Qt::darkRed, arm_control_graphics_scene_.get(), ui_.statusLabelArmControl);
            writeLog("Request timed out", ui_.logConsoleArmControl);
        }
    }
    void AvenaView::startArmController()
    {
        RCLCPP_INFO(node_->get_logger(), "Starting arm controller");
        sendArmCommand(ControlCommands::INIT);
    }
    void AvenaView::stopArmController()
    {
        RCLCPP_INFO(node_->get_logger(), "Stoping arm controller");

        sendArmCommand(ControlCommands::STOP);
    }
    void AvenaView::resumeArmController()
    {
        RCLCPP_INFO(node_->get_logger(), "Resuming arm controller");

        sendArmCommand(ControlCommands::RESUME);
    }
    void AvenaView::pauseArmController()
    {
        RCLCPP_INFO(node_->get_logger(), "Pause arm controller");

        sendArmCommand(ControlCommands::PAUSE);
    }
    void AvenaView::executeArmController()
    {
        RCLCPP_INFO(node_->get_logger(), "Execute arm controller trajectory");

        sendArmCommand(ControlCommands::EXECUTE);
    }

#pragma endregion

#pragma region SUBSCRIBERS

    void AvenaView::subscribeBtQuestion()
    {
        RCLCPP_INFO(node_->get_logger(), "Creating bt_question subscriber");

        bt_question_sub_ = node_->create_subscription<custom_interfaces::msg::GuiBtMessage>(
            "/bt_question",
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&AvenaView::btQuestionCallback, this, std::placeholders::_1));
    }

    void AvenaView::subscribeHeartBeat()
    {
        RCLCPP_INFO(node_->get_logger(), "Creating heartbeat subscriber");

        heartbeat_checker_ = node_->create_subscription<custom_interfaces::msg::Heartbeat>(
            "/system_monitor/heartbeat",
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&AvenaView::heartBeatCallback, this, std::placeholders::_1));
    }

    void AvenaView::subscribeLogs()
    {
        RCLCPP_INFO(node_->get_logger(), "Creating logs subscriber");

        logs_sub_ = node_->create_subscription<std_msgs::msg::String>(
            "/logs",
            rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&AvenaView::logsCallback, this, std::placeholders::_1));
    }

    void AvenaView::subscribeRosOut()
    {
        try
        {
            RCLCPP_INFO(node_->get_logger(), "Creating rosout subscriber");
            rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
                "/rosout",
                rclcpp::QoS(rclcpp::KeepLast(1)),
                std::bind(&AvenaView::rosoutCallback, this, std::placeholders::_1));
        }
        catch (rclcpp::exceptions::RCLError &e)
        {
            RCLCPP_ERROR(node_->get_logger(), e.what());
            RCLCPP_ERROR(node_->get_logger(), e.formatted_message);
            RCLCPP_ERROR(node_->get_logger(), e.file);
        }
    }

#pragma endregion

#pragma region SECURITY_STATUS

    void AvenaView::refreshDangerToolStatus(bool msg)
    {
        if (msg)
        {
            ui_.dangerToolLabel->setText("danger_tool_in_hand: ON");
            ui_.dangerToolLabel->setStyleSheet("QLabel {color: green;}");
        }
        else
        {
            ui_.dangerToolLabel->setText("danger_tool_in_hand: OFF");
            ui_.dangerToolLabel->setStyleSheet("QLabel {color: red;}");
        }
    }
    void AvenaView::refreshSecurityPasueStatus(bool msg)
    {
        if (msg)
        {
            ui_.securityPauseLabel->setText("security_pause: ON");
            ui_.securityPauseLabel->setStyleSheet("QLabel {color: green;}");
        }
        else
        {
            ui_.securityPauseLabel->setText("security_pause: OFF");
            ui_.securityPauseLabel->setStyleSheet("QLabel {color: red;}");
        }
    }
    void AvenaView::refreshSecurityTriggerStatus(bool msg)
    {
        if (msg)
        {
            ui_.securityTriggerLabel->setText("security_trigger: ON");
            ui_.securityTriggerLabel->setStyleSheet("QLabel {color: green;}");
        }
        else
        {
            ui_.securityTriggerLabel->setText("security_trigger: OFF");
            ui_.securityTriggerLabel->setStyleSheet("QLabel {color: red;}");
        }
    }

#pragma endregion

#pragma region

    void AvenaView::refreshNodeList(custom_interfaces::msg::Heartbeat::SharedPtr msg)
    {
        NodeListMap::iterator it = node_list_.find(msg->module_name);
        if (it != node_list_.end())
        {
            it->second = msg;
        }
        else
        {
            node_list_.insert({msg->module_name, msg});
        }
    }
    void AvenaView::refreshLogConsole(std_msgs::msg::String::SharedPtr msg)
    {
        QString q_msg = QString(msg->data.c_str());
        writeLog(q_msg, ui_.logConsoleArmControl);
    }
    void AvenaView::refreshRosoutConsole(rcl_interfaces::msg::Log::SharedPtr msg)
    {
        QString q_msg = QString(msg->msg.c_str());
        writeLog(q_msg, ui_.logConsoleCalibrate);
    }

#pragma endregion

#pragma region CALLBACKS
    void AvenaView::guiWarningCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !previus_gui_warning_msg_)
            emit securityWarningRecived();
        else if (!msg->data && previus_gui_warning_msg_)
            emit securityWarningClosed();

        previus_gui_warning_msg_ = msg->data;
    }

    void AvenaView::dangerToolStatusCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        emit dangerToolStatusChanged(msg->data);
    }

    void AvenaView::securityTriggerStatusCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        emit securityTriggerChanged(msg->data);
    }
    void AvenaView::securityPauseStatusCallback(std_msgs::msg::Bool::SharedPtr msg)
    {
        emit securityPauseChanged(msg->data);
    }

    void AvenaView::btQuestionCallback(custom_interfaces::msg::GuiBtMessage::SharedPtr msg)
    {
        emit questionRecived(msg);
    }

    void AvenaView::heartBeatCallback(custom_interfaces::msg::Heartbeat::SharedPtr msg)
    {
        emit nodeListChanged(msg);
    }

    void AvenaView::logsCallback(std_msgs::msg::String::SharedPtr msg)
    {
        emit logsAppeared(msg);
    }

    void AvenaView::rosoutCallback(rcl_interfaces::msg::Log::SharedPtr msg)
    {
        emit rosOutAppeared(msg);
    }

#pragma endregion

#pragma region UI_SETUP
    void AvenaView::setUpPickPlaceComboBoxes()
    {
        ui_.pickTargetCombo->setEditable(false);
        ui_.placeTargetCombo->setEditable(false);
        try
        {
            std::string parameters_package_path = ament_index_cpp::get_package_share_directory("parameters_server");
            std::string labels_path = parameters_package_path + "/config/labels";
            for (const auto &entry : fs::directory_iterator(labels_path))
            {
                if (isItem(entry.path()))
                {
                    std::string item_name = entry.path().filename();
                    item_name.erase(item_name.end() - 5, item_name.end());
                    ui_.pickTargetCombo->addItem(QString(item_name.c_str()));
                }

                if (isContrainer(entry.path()))
                {
                    std::string item_name = entry.path().filename();
                    item_name.erase(item_name.end() - 5, item_name.end());
                    ui_.placeTargetCombo->addItem(QString(item_name.c_str()));
                }
            }
            std::string areas_file_path = parameters_package_path + "/config/parameters/globals/areas.yaml";
            std::vector<std::string> area_names = getAreaNames(areas_file_path);
            for (const auto &area : area_names)
            {
                ui_.placeTargetCombo->addItem(QString(area.c_str()));
            }
        }
        catch (const std::exception &e)
        {
            writeTerminalAndUiLog(e.what(), Status::ERROR, ui_.logConsole);
            pick_place_status_ = Status::ERROR;
            return;
        }
    }

    void AvenaView::setUpStoppedPickPlaceUi()
    {
        ui_.startButton->setEnabled(true);
        ui_.stopButton->setEnabled(false);
        ui_.pauseButton->setEnabled(false);
        ui_.resumeButton->setEnabled(false);
        ui_.pickTargetCombo->setEnabled(false);
        ui_.placeTargetCombo->setEnabled(false);
        ui_.executeButton->setEnabled(false);
        pick_place_status_ = Status::STOPPED;
    }

    void AvenaView::setUpStartedPickPlaceUi()
    {
        ui_.startButton->setEnabled(false);
        ui_.stopButton->setEnabled(true);
        ui_.pauseButton->setEnabled(true);
        ui_.resumeButton->setEnabled(true);
        ui_.pickTargetCombo->setEnabled(true);
        ui_.placeTargetCombo->setEnabled(true);
        ui_.executeButton->setEnabled(true);
        pick_place_status_ = Status::RUNNING;
    }
#pragma endregion

#pragma region PICK_PLACE_ACTION

    void AvenaView::executePickPlace()
    {
        sendPickPlaceGoal("start");
    }

    void AvenaView::sendPickPlaceGoal(const std::string &command)
    {
        writeLog("Running pick place BT", ui_.logConsole);
        using namespace std::placeholders;
        if (!this->pick_place_action_client_->wait_for_action_server(100ms))
        {
            RCLCPP_ERROR(node_->get_logger(), "Action server not available");
            rclcpp::shutdown();
        }

        auto goal_msg = BTPickAndPlaceAction::Goal();
        goal_msg.command = command;
        goal_msg.what_to_pick = ui_.pickTargetCombo->currentText().toUtf8().constData();
        goal_msg.where_to_put = ui_.placeTargetCombo->currentText().toUtf8().constData();

        RCLCPP_INFO(node_->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<BTPickAndPlaceAction>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&AvenaView::pickPlaceFeedbackCallback, this, _1, _2);
        send_goal_options.goal_response_callback = std::bind(&AvenaView::pickPlaceGoalResponseCallback, this, _1);
        send_goal_options.result_callback = std::bind(&AvenaView::pickPlaceResultCallback, this, _1);

        this->pick_place_action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void AvenaView::pickPlaceGoalResponseCallback(std::shared_future<GoalHandleBTPickAndPlaceAction::SharedPtr> future)
    {
    }

    void AvenaView::pickPlaceFeedbackCallback(
        GoalHandleBTPickAndPlaceAction::SharedPtr,
        const std::shared_ptr<const BTPickAndPlaceAction::Feedback> feedback)
    {
    }

    void AvenaView::pickPlaceResultCallback(const GoalHandleBTPickAndPlaceAction::WrappedResult &result)
    {
        writeLog((result.result->result) ? "Finnished with: success" : "Finished with: failure", ui_.logConsole);
    }

    void AvenaView::pausePickPlace()
    {
        writeLog("Canceling pick place BT", ui_.logConsole);
        pick_place_action_client_->async_cancel_all_goals();
    }

    void AvenaView::cancelPickPlace()
    {
        writeLog("Canceling pick place BT", ui_.logConsole);
        pick_place_action_client_->async_cancel_all_goals();
        sendPickPlaceGoal("stop");
    }

#pragma endregion

#pragma region BT_USER_INTERACTION

    void AvenaView::showQuestionMessageBox(custom_interfaces::msg::GuiBtMessage::SharedPtr msg)
    {
        QMessageBox msg_box;
        QPushButton *ok_btn = msg_box.addButton(tr("OK"), QMessageBox::ActionRole);
        QPushButton *cancel_btn = msg_box.addButton(tr("CANCEL"), QMessageBox::ActionRole);
        msg_box.setText(tr(msg->msg.c_str()));
        msg_box.exec();

        if (msg_box.clickedButton() == ok_btn)
        {
            msg->msg = "true";
            writeLog("OK", ui_.logConsole);
        }
        else if (msg_box.clickedButton() == cancel_btn)
        {
            msg->msg = "false";
            writeLog("CANCEL", ui_.logConsole);
        }
        user_answer_pub_->publish(*msg);
    }

    void AvenaView::showSecurityRgbWarning()
    {
        writeLog("Security RGB Warning", ui_.logConsole);
        security_rgb_warning_->exec();

        // auto q_progress = std::make_shared<QProgressDialog>(
        //     "Securit4y area breached. Waiting 5s.", "Abort", 0, BT_WARNING_DURATION);

        // q_progress->setCancelButton(nullptr);

        // q_progress->setValue(0);
        // q_progress->show();

        // for (size_t i = 0; i < BT_WARNING_DURATION; i++)
        // {
        //     QCoreApplication::processEvents();
        //     q_progress->setValue(i);
        //     std::this_thread::sleep_for(std::chrono::seconds(1));
        // }

        // q_progress->setValue(BT_WARNING_DURATION);
    }

    void AvenaView::hideSecurityRgbWarning()
    {
        security_rgb_warning_->close();
    }

#pragma endregion

#pragma region PUBLISH_RGB_DIFF_CONTROL_PANEL
    void AvenaView::publishSetBackgroundSignal()
    {
        std_msgs::msg::Bool signal;
        signal.data = true;
        set_background_pub_->publish(signal);
        writeLog("Setting background for security", ui_.logConsole);
    }
    void AvenaView::publishSceneChangeThreshold()
    {
        std_msgs::msg::Int32 msg;
        msg.data = ui_.sceneChangeTresholdInput->value();
        scene_change_treshold_pub_->publish(msg);
        writeLog("Setting scene chage threshold to " + QString::number(ui_.sceneChangeTresholdInput->value()), ui_.logConsole);
    }
    void AvenaView::publishPixelThreshold()
    {
        std_msgs::msg::Int32 msg;
        msg.data = ui_.perPixelTresholdInput->value();
        pixel_treshold_pub_->publish(msg);
        writeLog("Setting pixel threshold to " + QString::number(ui_.perPixelTresholdInput->value()), ui_.logConsole);
    }
#pragma endregion

#pragma region CALIBRATION

    void AvenaView::runCalibration()
    {
        QMessageBox msg_box;
        QPushButton *done_btn = msg_box.addButton(tr("DONE"), QMessageBox::ActionRole);
        QPushButton *abort_btn = msg_box.addButton(tr("ABORT"), QMessageBox::ActionRole);
        msg_box.setText("Attach calibration chessbord to robotic arm");
        msg_box.exec();

        if (msg_box.clickedButton() == done_btn)
        {
            runCalibrationLaunchFile();
        }
        else if (msg_box.clickedButton() == abort_btn)
        {
            stopCalibrate();
        }
    }

    void AvenaView::stopCalibrate()
    {
        RCLCPP_INFO(node_->get_logger(), "Stoping system");

        if (calibrate_launch_file_pid_ > 0)
        {
            //TODO: change returend value type to int with exit code
            if (killAllChildProcessPids(calibrate_launch_file_pid_))
            {
                writeTerminalAndUiLog("Sucessfully stopped calibration", Status::STOPPED, ui_.logConsole);
            }
            else
            {
                writeTerminalAndUiLog("Error while stopping calibration", Status::ERROR, ui_.logConsole);
            }
        }
        else
        {
            writeTerminalAndUiLog("Nothing to stop", Status::ERROR, ui_.logConsole);
        }
    }

    void AvenaView::calibrateGoalResponseCallback(std::shared_future<GoalCalibrateAction::SharedPtr> future)
    {
    }

    void AvenaView::calibrateFeedbackCallback(
        GoalCalibrateAction::SharedPtr,
        const std::shared_ptr<const CalibrateAction::Feedback> feedback)
    {
    }

    void AvenaView::calibrateResultCallback(const GoalCalibrateAction::WrappedResult &result)
    {
        this->writeLog("Calibration action returned result", ui_.logConsoleCalibrate);
    }

    // FIXME: refactor this
    void AvenaView::runCalibrationLaunchFile()
    {
        using namespace std::placeholders;
        RCLCPP_INFO(node_->get_logger(), "Starting system");
        QString program = "ros2";
        calibrate_launch_file_process_->setArguments({"launch", "avena_bringup", CALIBRATE_LAUNCH_FILE});
        calibrate_launch_file_process_->setProgram(program);
        calibrate_launch_file_pid_ = 0;

        if (calibrate_launch_file_process_->startDetached(&calibrate_launch_file_pid_))
        {
            this->writeLog("Starting calibration", ui_.logConsoleCalibrate);
            auto post_start_action = [this]()
            {
                if (!this->calibrate_action_client_->wait_for_action_server(200ms))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Action server not available");
                    rclcpp::shutdown();
                }
                auto goal_msg = CalibrateAction::Goal();
                auto send_goal_options = rclcpp_action::Client<CalibrateAction>::SendGoalOptions();
                send_goal_options.feedback_callback = std::bind(&AvenaView::calibrateFeedbackCallback, this, _1, _2);
                send_goal_options.goal_response_callback = std::bind(&AvenaView::calibrateGoalResponseCallback, this, _1);
                send_goal_options.result_callback = std::bind(&AvenaView::calibrateResultCallback, this, _1);

                this->calibrate_action_client_->async_send_goal(goal_msg, send_goal_options);

                this->writeLog("Sucessfully started calibration", ui_.logConsoleCalibrate);
            };
            QTimer::singleShot(3000, this, post_start_action);
        }
        else
        {
            writeLog("Error while starting calibration", ui_.logConsoleCalibrate);
        }
    }

#pragma endregion
}

PLUGINLIB_EXPORT_CLASS(avena_view::AvenaView, rqt_gui_cpp::Plugin)