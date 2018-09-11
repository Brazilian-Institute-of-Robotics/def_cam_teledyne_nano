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

class App {
public:
    App();
    ~App();
    void run(std::string camera_username, int argc, char **argv);
    void receivedFrame(teledyne::Frame* frame);
    void saveFrame(cv::Mat mat);
    void frameToMat(teledyne::Frame frame, cv::Mat& dst);
    void startStream();
    cv::Mat* getCurrentImage();
    /* This method should be static*/
    static void receivedFrameCallback(teledyne::Frame frame, void *params);
private:
    teledyne::Camera camera;
    cv::Mat out;
    ros::NodeHandle nh;
    image_transport::Publisher pub;
};
#endif // APPLICATION_H
