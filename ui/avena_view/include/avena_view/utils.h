#pragma once
#include <string>
#include <stdexcept>
#include <initializer_list>
#include <QTextBrowser>
#include <sstream>
#include <signal.h>
#include <iostream>
#include <builtin_interfaces/msg/time.hpp>
#include <QTime>
#include <QCoreApplication>


std::string exec(const char *cmd);

std::string joinPath(std::initializer_list<const std::string> args);

int killAllChildProcessPids(qint64 launch_file_pid);

std::chrono::nanoseconds rosTime2Chrono(builtin_interfaces::msg::Time &stamp);

void writeToConsole(const std::string &msg, QTextBrowser *dst_ptr);

void delay(int milliseconds);
