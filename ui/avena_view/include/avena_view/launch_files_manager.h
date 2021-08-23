#pragma once
#include <QProcess>
#include <fstream>
#include <memory>
#include <map>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <avena_view/utils.h>
#include <QTimer>
#include <filesystem>

using PID = qint64;
namespace fs = std::filesystem;

class LaunchFile : public QObject
{
    Q_OBJECT

public:
    explicit LaunchFile(const std::string &pid_file_name);
    ~LaunchFile();

    template <typename F1>
    bool run(F1 post_start_action, int post_start_delay)
    {
        bool restoring_result = restoreSessionFromFile();

        if (restoring_result)
        {
            return true;
        }

        if (args_.size() == 0)
        {
            throw std::runtime_error(std::string("args vector is empty"));
        }

        launch_file_process_->setArguments(args_);
        bool lambda_return_value = false;
        auto lambda = [post_start_action, &lambda_return_value]()
        { lambda_return_value = post_start_action(); };

        if (launch_file_process_->startDetached(&this->pid_))
        {
            out_pid_file_ << this->pid_;
            delay(post_start_delay);
            lambda();
            // QTimer::singleShot(post_start_delay, this, SLOT(lambda()));
            if(!lambda_return_value)
            {
                std::cout << "Module was not correctly initialized" << std::endl;
                return false;
            }
            std::cout << "Successfully started launch file: " << pid_file_name_ << std::endl;
            std::cout << "PID: " << pid_ << std::endl;
            return true;
        }
        else
        {
            std::cout << "Error while running launch file: " << pid_file_name_ << std::endl;
            pid_ = 0;
            return false;
        }
    }

    bool terminate();
    bool restoreSessionFromFile();
    void setArguments(const QStringList args);
    void removePidFile();

    bool isRunning();

private:
    PID pid_;
    std::shared_ptr<QProcess> launch_file_process_;
    std::string pid_file_name_;
    std::ifstream in_pid_file_;
    std::ofstream out_pid_file_;
    QStringList args_;
    std::string pid_file_path_;
};

class LaunchFileManager : public QObject
{
    Q_OBJECT
public:
    const std::map<std::string, std::shared_ptr<LaunchFile>> &launchFiles() const { return launch_files_; }
    void addLaunch(const std::string &launch_name);

    template <typename F1>
    bool runLaunch(const std::string &launch_name, F1 post_start_action, int post_start_delay)
    {
        auto it = launch_files_.find(launch_name);
        if (it == launch_files_.end())
        {
            std::cout << "Launch " << launch_name << " not found" << std::endl;
            return false;
        }

        return it->second->run(post_start_action, post_start_delay);
    }
    bool terminateLaunch(const std::string &launch_name);
    bool removeLaunch(const std::string &launch_name);

private:
    std::map<std::string, std::shared_ptr<LaunchFile>> launch_files_;
};
