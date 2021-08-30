#pragma once

#include <QObject>
#include <avena_view/launch_files_manager.h>

#include <ui_avena_view.h>
#include <rclcpp/rclcpp.hpp>

#include "avena_view/utils.h"
#include "avena_view/nodes_list.h"
// #include "avena_view/detectron_runner.h"

class StartSystem : public QObject
{
    Q_OBJECT

    public:
        StartSystem(Ui::AvenaViewWidget *ui_ptr, rclcpp::Node::SharedPtr node_shared_ptr, NodesMap& nodes_map_ref);
        ~StartSystem();

    private slots:
        void startCamera();
        void startCAN();
        void startArmController();
        void startBullet();
        void startGeneratePath();
        void startGenerateTrajectory();

        void startVisionSystem();
        void startParametersServer();
        void startDataStore();
        void startDetect();
        void startSecurityRGB();
        void startTrackingRGB();
        void startRGBDSync();
        void startComposeItems();
        void startEstimateShape();
        void startGenerateOctomap();

    private:
        void connectSlotsToSignals();
        bool startNode(const QString name,const QStringList args, unsigned int ms_waiting=3000);
        void addLaunches();

        std::shared_ptr<LaunchFileManager> _launch_file_manager;
        Ui::AvenaViewWidget *_ui_ptr; 
        rclcpp::Node::SharedPtr _node_shared_ptr;
        NodesMap& _nodes_map_ref;
        // std::shared_ptr<DetectronRunner> _detectron_runner;
};