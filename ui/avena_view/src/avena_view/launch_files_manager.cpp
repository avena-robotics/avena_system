#include "avena_view/launch_files_manager.h"

LaunchFile::LaunchFile(const std::string &pid_file_name)
    : pid_file_name_(pid_file_name), pid_(0)
{
    launch_file_process_ = std::make_shared<QProcess>();
    launch_file_process_->setProgram("ros2");
    pid_file_path_ = joinPath({ament_index_cpp::get_package_share_directory("avena_view"), pid_file_name});
    in_pid_file_.open(pid_file_path_);
    out_pid_file_.open(pid_file_path_);
}

LaunchFile::~LaunchFile()
{
    if (in_pid_file_.is_open())
        in_pid_file_.close();

    if (out_pid_file_.is_open())
        out_pid_file_.close();
}

void LaunchFile::setArguments(const QStringList args)
{
    args_ = args;
}

bool LaunchFile::restoreSessionFromFile()
{
    std::cout << "Restoring running launch file info: " << pid_file_name_ << std::endl;
    if (!in_pid_file_)
    {
        std::cout << "No saved launch file info" << std::endl;
        return false;
    }

    in_pid_file_ >> pid_;
    if (pid_ > 0)
    {
        std::cout << "Restored launch file info: " << pid_file_name_ << std::endl;
        std::cout << "PID: " << pid_ << std::endl;
        return true;
    }
    return false;
}

bool LaunchFile::terminate()
{
    if (pid_ > 0)
    {
        int killing_result = killAllChildProcessPids(pid_);
        if (killing_result == 0)
        {
            pid_ = 0;
            if (fs::remove(pid_file_path_))
            {
                std::cout << "Sucessfully stoped launch file and removed pid file" << std::endl;
                return true;
            }
            else
            {
                std::cout << "Can not delete pid file: " << pid_file_path_ << std::endl;
                return false;
            }
        }
    }
    else
    {
        std::cout << "Nothing to stop" << std::endl;
        return false;
    }
}

bool LaunchFile::isRunning()
{
    return pid_ == 0;
}

void LaunchFileManager::addLaunch(const std::string &launch_name)
{
    launch_files_.insert({launch_name, std::make_shared<LaunchFile>(launch_name)});
}


bool LaunchFileManager::terminateLaunch(const std::string &launch_name)
{
    auto it = launch_files_.find(launch_name);
    if (it == launch_files_.end())
    {
        std::cout << "Launch " << launch_name << " not found" << std::endl;
        return false;
    }

    return it->second->terminate();
}

bool LaunchFileManager::removeLaunch(const std::string &launch_name)
{
    auto it = launch_files_.find(launch_name);
    if (it == launch_files_.end())
    {
        std::cout << "Launch " << launch_name << " not found" << std::endl;
        return false;
    }

    bool terminating_result = true;

    if (it->second->isRunning())
    {
        std::cout << "Launch file you trying to remove is still running. Terminating." << std::endl;
        terminating_result = it->second->terminate();
    }

    if (terminating_result)
    {
        if(launch_files_.erase(it->first)==1)
            return true;
        else
        {
            std::cout << "Error while removing launch file" << std::endl;
            return false;
        }
    }
    else
    {
        std::cout << "Error while terminating launch file" << std::endl;
        return false;
    }
}
