#ifndef CAMERA_PUBLISHER_HPP_
#define CAMERA_PUBLISHER_HPP_

#include "CameraAPI.hpp"

#include <iostream>
#include <string>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>


class CameraPublisher
{
  public:
    CameraPublisher(const ros::NodeHandle &nodeHandle,
                    const std::string &camera_serial_number,
                    const std::string &path_to_config,
                    const std::string &topic_name,
                    const std::string &camera_name="teledyne_nano");
    ~CameraPublisher();
    void startStreaming();
    void stopStreaming();
    void receivedFrame(teledyne::Frame *frame);
    static void receivedFrameCallback(teledyne::Frame frame, void *params);
    void frameToMat(teledyne::Frame frame, cv::Mat &dst);
    void saveFrame(const cv::Mat &frame);
    void showFrames(const cv::Mat &frame);
    void publishFrame(const cv::Mat &frame);

  private:
    std::shared_ptr<teledyne::Camera> cameraPtr;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cameraInfoPtr;
    image_transport::CameraPublisher imagePublisher;
};

#endif // APPLICATION_HPP_
