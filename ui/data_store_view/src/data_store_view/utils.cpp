#pragma once

#include <data_store_view/utils.h>
#include <QTime>
#include <QCoreApplication>


void rosImageToQt(const sensor_msgs::msg::Image &image, QImage &out_image)
{
    cv::Mat cv_image;
    helpers::converters::rosImageToCV(image, cv_image);
    // std::cout << cv_image.size() << std::endl;
    // std::cout << cv_image.type() << std::endl;
    // std::cout << cv_image.step1() << std::endl;
    cv::cvtColor(cv_image, cv_image, CV_BGR2RGBA);
    out_image = QImage(cv_image.data, cv_image.cols, cv_image.rows, QImage::Format_RGBA8888);


}

void delay(int milliseconds)
{
    QTime dieTime = QTime::currentTime().addMSecs(milliseconds);
    while (QTime::currentTime() < dieTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

std::string geometryMsgPoseToString(const geometry_msgs::msg::Pose& pose)
{
    std::string result = "";
    result += "position:\n";
    result += "\tx:" + std::to_string(pose.position.x) + "\n";
    result += "\ty:" + std::to_string(pose.position.y) + "\n";
    result += "\tz:" + std::to_string(pose.position.z) + "\n";
    result += "orientation:\n";
    result += "\tx:" + std::to_string(pose.orientation.x) + "\n";
    result += "\ty:" + std::to_string(pose.orientation.y) + "\n";
    result += "\tz:" + std::to_string(pose.orientation.z) + "\n";
    result += "\tw:" + std::to_string(pose.orientation.w) + "\n";
    return result;
}