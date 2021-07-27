#pragma once
#include <string>
#include <stdexcept>
#include <initializer_list>
#include <QObject>
#include <sstream>
#include <signal.h>
#include <iostream>

std::string exec(const char *cmd);

std::string joinPath(std::initializer_list<const std::string> args);

int killAllChildProcessPids(qint64 launch_file_pid);