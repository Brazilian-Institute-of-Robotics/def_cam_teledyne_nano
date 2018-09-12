#ifndef APPLICATION_H
#define APPLICATION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <camera.hpp>

#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>

class App
{
  public:
    App(ros::NodeHandle nh);
    ~App();
    void run(std::string camera_username, int argc, char **argv);
    void receivedFrame(teledyne::Frame *frame);
    void saveFrame(cv::Mat mat);
    void frameToMat(teledyne::Frame frame, cv::Mat &dst);
    void startStream();
    cv::Mat *getCurrentImage();
    /* This method should be static*/
    static void receivedFrameCallback(teledyne::Frame frame, void *params);

  private:
    teledyne::Camera camera;
    cv::Mat out;
    ros::NodeHandle nh_;
    image_transport::ImageTransport *it_;
    image_transport::CameraPublisher image_pub_;

    camera_info_manager::CameraInfoManager *cinfo_;
    std::string camera_name_;
};

#endif // APPLICATION_H
