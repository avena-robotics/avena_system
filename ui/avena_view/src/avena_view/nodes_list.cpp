#include "avena_view/nodes_list.h"

NodesList::NodesList(Ui::AvenaViewWidget *ui_ptr, rclcpp::Node::SharedPtr node_shared_ptr)
    : _ui_ptr(ui_ptr), _node_shared_ptr(node_shared_ptr)
{
    qRegisterMetaType<custom_interfaces::msg::Heartbeat::SharedPtr>("custom_interfaces::msg::Heartbeat::SharedPtr");

    _refresh_nodes_map_timer = std::make_shared<QTimer>();

    connectSignalsToSlots();
    setUpRosComponents();

    _refresh_nodes_map_timer->start(GUI_REFRESH_DELAY);
}

NodesList::~NodesList() {
    std::cout << __func__ << std::endl;
}

void NodesList::setUpRosComponents()
{
    _heartbeat_checker = _node_shared_ptr->create_subscription<custom_interfaces::msg::Heartbeat>(
        "/system_monitor/heartbeat",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&NodesList::heartBeatCallback, this, std::placeholders::_1));

    _command_publisher = _node_shared_ptr->create_publisher<custom_interfaces::msg::ModuleCommand>("/system_monitor/command", 10);
}

void NodesList::connectSignalsToSlots()
{
    connect(_refresh_nodes_map_timer.get(), SIGNAL(timeout()), this, SLOT(refreshGUI()));
    connect(this, SIGNAL(nodeListChanged(custom_interfaces::msg::Heartbeat::SharedPtr)), this, SLOT(refreshNodesMap(custom_interfaces::msg::Heartbeat::SharedPtr)));
    connect(_ui_ptr->nodeListTable, SIGNAL(cellClicked(int, int)), this, SLOT(handleTableCellClick(int, int)));
}

void NodesList::refreshNodesMap(custom_interfaces::msg::Heartbeat::SharedPtr msg)
{
    NodesMap::iterator it = nodes_map.find(msg->module_name);
    if (it != nodes_map.end())
    {
        it->second = msg;
    }
    else
    {
        nodes_map.insert({msg->module_name, msg});
    }
}

void NodesList::setUpTabContent(const NodeType &node, int row)
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

    _ui_ptr->nodeListTable->setItem(row, 0, q_name);
    _ui_ptr->nodeListTable->setItem(row, 1, q_since_last);
    _ui_ptr->nodeListTable->setItem(row, 2, q_status);
    _ui_ptr->nodeListTable->setItem(row, 3, q_command_button);
}

void NodesList::heartBeatCallback(custom_interfaces::msg::Heartbeat::SharedPtr msg)
{
    emit nodeListChanged(msg);
}

void NodesList::refreshGUI()
{
    if (_refreshing_on)
    {
        _ui_ptr->nodeListTable->setRowCount(nodes_map.size());
        int row = 0;
        for (const auto &node : nodes_map)
        {
            setUpTabContent(node, row++);
        }
    }
}

void NodesList::handleTableCellClick(int row, int col)
{
    if (col != COMMAND_COL)
        return;

    std::string command = _ui_ptr->nodeListTable->item(row, COMMAND_COL)->text().toStdString();
    std::string module_name = _ui_ptr->nodeListTable->item(row, NODE_NAME_COL)->text().toStdString();
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
        writeToConsole("Unrecognized command\n", _ui_ptr->nodesLogConsole);
        return;
    }
    module_command->header = std_msgs::msg::Header();
    module_command->header.stamp = _node_shared_ptr->now();
    module_command->module_name = module_name;

    _command_publisher->publish(*module_command);
    writeToConsole("Sent command: " + command + " to: " + module_name + " module\n", _ui_ptr->nodesLogConsole);
}
