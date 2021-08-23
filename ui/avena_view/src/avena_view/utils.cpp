#include "avena_view/utils.h"

std::string exec(const char *cmd)
{
    char buffer[128];
    std::string result = "";
    FILE *pipe = popen(cmd, "r");
    if (!pipe)
        throw std::runtime_error("popen() failed!");
    try
    {
        while (fgets(buffer, sizeof buffer, pipe) != NULL)
        {
            result += buffer;
        }
    }
    catch (...)
    {
        pclose(pipe);
        throw;
    }
    pclose(pipe);
    return result;
}

std::string joinPath(std::initializer_list<const std::string> args)
{
    std::string result = "";
    for (auto arg : args)
    {
        result += arg;
        result += "/";
    }
    return result.substr(0, result.size() - 1);
}

void delay(int milliseconds)
{
    QTime dieTime = QTime::currentTime().addMSecs(milliseconds);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

int killAllChildProcessPids(qint64 launch_file_pid)
{
    std::string pids = exec("ps -e -o ppid= -o pid=");
    std::stringstream ss;
    ss << pids;

    qint64 pid, ppid;
    std::map<qint64, std::vector<qint64>> pids_map;
    std::map<qint64, std::vector<qint64>>::iterator it;
    while (ss >> ppid >> pid)
    {
        it = pids_map.lower_bound(ppid);
        if (it != pids_map.end())
            pids_map[ppid].push_back(pid);
        else
            pids_map.insert({ppid, {pid}});
    }

    std::map<int, std::vector<qint64>> pids_layers;
    int layer_id = 0;
    pids_layers.insert({layer_id++, {launch_file_pid}});
    pids_layers.insert({layer_id, pids_map[launch_file_pid]});
    int killing_result;
    int final_result = 0;
    for (int i = pids_layers.size() - 1; i >= 0; i--)
    {
        for (int j = 0; j < pids_layers[i].size(); j++)
        {
            killing_result = kill(pids_layers[i][j], SIGINT);
            if (killing_result != 0)
            {
                final_result = killing_result;
                std::cout << "Error while killing process with pid=" << pids_layers[i][j] << std::endl;
                std::cout << "Error code: " << killing_result << std::endl;
            }
        }
    }
    return final_result;
}

std::chrono::nanoseconds rosTime2Chrono(builtin_interfaces::msg::Time &stamp)
{
    auto seconds = std::chrono::seconds(stamp.sec);
    auto nanoseconds = std::chrono::nanoseconds(stamp.nanosec);

    return seconds + nanoseconds;
}

void writeToConsole(const std::string &msg, QTextBrowser *dst_ptr)
{
    dst_ptr->append(QString(msg.c_str()));
}