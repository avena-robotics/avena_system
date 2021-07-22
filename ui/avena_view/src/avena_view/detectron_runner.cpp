#include "avena_view/detectron_runner.h"

DetectronRunner::DetectronRunner(Ui::AvenaViewWidget* ui)
{
    share_dir_path_ = ament_index_cpp::get_package_share_directory("detect_server");
    QString path_to_script = QString(share_dir_path_.c_str()) + "/../../lib/detect_server/detect_server";
    std::cout << "Path to script: " << path_to_script.toUtf8().toStdString() << std::endl;
    detectron_process_ = std::make_shared<QProcess>();
    detectron_process_->setProgram("python3");
    detectron_process_->setArguments({path_to_script});
    detectron_process_->setStandardOutputFile( QString(share_dir_path_.c_str()) + "/detectron_output.txt");
    detectron_process_->setStandardErrorFile( QString(share_dir_path_.c_str()) + "/detectron_output.txt");
    pid_ = 0;
    out_pid_file_.open( share_dir_path_ + "/detect.pid");
    in_pid_file_.open( share_dir_path_ + "/detect.pid");
    in_logs_file_.open( share_dir_path_ + "/detectron_output.txt" );
    ui_ = ui;

    logs_file_watcher_ = std::make_shared<QFileSystemWatcher>();
    logs_file_watcher_->addPath(QString(share_dir_path_.c_str()) + "/detectron_output.txt");

    connect(ui_->detectronStartButton, SIGNAL(clicked(bool)), this, SLOT(startDetectron()));
    connect(ui_->detectronStopButton, SIGNAL(clicked(bool)), this, SLOT(stopDetectron()));
    connect(logs_file_watcher_.get(), SIGNAL(fileChanged(QString)), this, SLOT(showLast20Logs()));
}

DetectronRunner::~DetectronRunner()
{

}

void DetectronRunner::startDetectron()
{
    bool restoring_result = restoreLastSession();

    if(restoring_result)
        return;

    if(!out_pid_file_)
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
    int killing_result = 0;
    if(pid_ != 0)
        killing_result = kill(pid_, SIGINT);
    ui_->detectronLogConsole->append( "Detectron stopped with exit code: " + QString::number(killing_result));
    fs::remove(share_dir_path_ + "/detect.pid");
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
        return false;
    }
    
    in_pid_file_ >> pid_;
    return true;
}

void DetectronRunner::showLast20Logs()
{
    in_logs_file_.clear();
    in_logs_file_.seekg(0, std::ios::beg);
    std::string line;
    ui_->detectronLogConsole->clear();
	while(std::getline(in_logs_file_,line))
    {
        ui_->detectronLogConsole->append(QString(line.c_str()));
    }
}
