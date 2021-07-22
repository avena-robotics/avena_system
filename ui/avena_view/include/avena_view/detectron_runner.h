#pragma once

#include <QProcess>
#include <QString>
#include <memory>
#include <fstream>
#include <QFileSystemWatcher>
#include <ui_avena_view.h>
#include <iostream>
#include "utils.h"
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <signal.h>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

class DetectronRunner : public QObject
{
    Q_OBJECT

    public:
        explicit DetectronRunner(Ui::AvenaViewWidget* ui);
        ~DetectronRunner();

    public slots:
        void startDetectron();
        void stopDetectron();
        void showLast20Logs();
    
    private:
        bool restoreLastSession();

        std::shared_ptr<QProcess> detectron_process_;
        std::shared_ptr<QFileSystemWatcher> logs_file_watcher_;
        std::ofstream out_pid_file_;
        std::ifstream in_logs_file_;
        std::ifstream in_pid_file_;
        qint64 pid_;
        std::string share_dir_path_;

        Ui::AvenaViewWidget* ui_;
};