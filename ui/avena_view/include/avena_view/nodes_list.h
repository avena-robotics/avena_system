#pragma once

#include <QTimer>

#include <ui_avena_view.h>

#include "custom_interfaces/msg/heartbeat.hpp"
#include "custom_interfaces/msg/module_command.hpp"

#include "avena_view/config.h"
#include "avena_view/utils.h"

#include <rclcpp/rclcpp.hpp>

Q_DECLARE_METATYPE(custom_interfaces::msg::Heartbeat::SharedPtr)

using NodeType = std::pair<std::string, custom_interfaces::msg::Heartbeat::SharedPtr>;
using NodesMap = std::map<std::string, custom_interfaces::msg::Heartbeat::SharedPtr>;

class NodesList : public QObject
{
    Q_OBJECT

public:
    NodesList(Ui::AvenaViewWidget *ui_ptr, rclcpp::Node::SharedPtr node_shared_ptr);
    ~NodesList();
    NodesMap nodes_map;

private slots:
    void handleTableCellClick(int row, int col);
    void refreshGUI();
    void refreshNodesMap(custom_interfaces::msg::Heartbeat::SharedPtr msg);

signals:
    void nodeListChanged(custom_interfaces::msg::Heartbeat::SharedPtr msg);

private:
    void heartBeatCallback(custom_interfaces::msg::Heartbeat::SharedPtr msg);
    void setUpTabContent(const NodeType &node, int row);
    void setUpRosComponents();

    void connectSignalsToSlots();

    Ui::AvenaViewWidget *_ui_ptr;
    rclcpp::Node::SharedPtr _node_shared_ptr;

    std::shared_ptr<QTimer> _refresh_nodes_map_timer;

    bool _refreshing_on;

    rclcpp::Subscription<custom_interfaces::msg::Heartbeat>::SharedPtr _heartbeat_checker;
    rclcpp::Publisher<custom_interfaces::msg::ModuleCommand>::SharedPtr _command_publisher;
};