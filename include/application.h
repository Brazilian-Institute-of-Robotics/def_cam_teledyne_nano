#ifndef APPLICATION_H
#define APPLICATION_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <camera.hpp>

class App {
public:
    App();
    ~App();
    void run(std::string camera_username);
    void receivedFrame(teledyne::Frame* frame);
    void saveFrame(cv::Mat mat);
    void frameToMat(teledyne::Frame frame, cv::Mat& dst);
    void startStream();
    /* This method should be static*/
    static void receivedFrameCallback(teledyne::Frame frame, void *params);
private:
    teledyne::Camera camera;
    cv::Mat out;
};
#endif // APPLICATION_H
