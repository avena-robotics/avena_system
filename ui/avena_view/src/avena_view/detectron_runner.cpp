#include "avena_view/detectron_runner.h"

DetectronRunner::DetectronRunner(Ui::AvenaViewWidget* ui)
{
    int dupa = 0;
    std::string share_dir_path = ament_index_cpp::get_package_share_directory("detect_server");
    
    std::cout << __func__ << " " << dupa++ << std::endl;
    QString path_to_script = QString(share_dir_path.c_str()) + "/../lib/detect_server/detect_server";

    std::cout << __func__ << " " << dupa++ << std::endl;
    std::cout << "Path to script: " << path_to_script.toUtf8().toStdString() << std::endl;

    std::cout << __func__ << " " << dupa++ << std::endl;
    detectron_process_ = std::make_shared<QProcess>();
    std::cout << __func__ << " " << dupa++ << std::endl;
    detectron_process_->setProgram("python3");
    std::cout << __func__ << " " << dupa++ << std::endl;
    detectron_process_->setArguments({path_to_script});
    std::cout << __func__ << " " << dupa++ << std::endl;
    pid_ = 0;
    out_pid_file_.open("detectron.pid");
    in_pid_file_.open("detectron.pid");
    std::cout << __func__ << " " << dupa++ << std::endl;

    ui_ = ui;
    std::cout << __func__ << " " << dupa++ << std::endl;
}

DetectronRunner::~DetectronRunner()
{

}

void DetectronRunner::startDetectron()
{
    if(out_pid_file_)
        throw std::runtime_error(std::string("Cannot open pid file"));
    if(detectron_process_->startDetached(&pid_))
    {
        out_pid_file_ << pid_;
        ui_->detectronLogConsole->append("Starting detect server");
    }
    else
    {
        ui_->detectronLogConsole->append("Failed to start detect server");
    }
    out_pid_file_.close();
}
void DetectronRunner::stopDetectron()
{
    if(kill(pid_, SIGINT) == 0)
    {
        std::cout << "Stopping detectron server" << std::endl;
    }
}

bool DetectronRunner::restoreLastSession()
{
    if(!in_pid_file_)
        return false;
    
    std::cout << "Restoring last detectron session" << std::endl;
    std::string gpu_pids = exec("ps f -o pid,command -p `lsof -n -w -t /dev/nvidia*` | awk '(NR>1)' | grep python");
    std::stringstream ss;
    ss << gpu_pids;
    std::string process_pid, process_name;
    int amount_of_gpu_process = 0;
    while(ss >> process_pid >> process_name)
    {
        amount_of_gpu_process++;
    }

    if(amount_of_gpu_process == 2)
    {
        std::cout << "Both instances of detectron seems to be still active" << std::endl;
    }
    else
    {
        std::cout << "Instances are missing" << std::endl;
    }
    
    in_pid_file_ >> pid_;
    return true;
}

void DetectronRunner::showLast20Logs()
{

}
