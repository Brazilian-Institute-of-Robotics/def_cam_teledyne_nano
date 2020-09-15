#include "CameraPublisher.hpp"


CameraPublisher::CameraPublisher(const ros::NodeHandle &node_handle,
         const std::string &camera_serial_number,
         const std::string &path_to_config,
         const std::string &topic_name,
         const std::string &camera_name)
{
    cameraInfoPtr = std::make_shared<camera_info_manager::CameraInfoManager>(
                node_handle,
                camera_name,
                path_to_config);
    imagePublisher = image_transport::ImageTransport(
                node_handle).advertiseCamera( topic_name, 1 );

    cameraPtr = std::make_shared<teledyne::Camera>(2592, 2048, fmtBayerRG8);
    cameraPtr->open(camera_serial_number);
    std::cout << "Opened" << std::endl;
    cameraPtr->setFPS(20);
    cameraPtr->setupCallback(CameraPublisher::receivedFrameCallback, this);
    cameraPtr->enableAutoWhiteBalance();
    cameraPtr->applyAutoWhiteBalance();

    ROS_INFO( "Width = %d", cameraPtr->getWidth());
    ROS_INFO( "Height = %d", cameraPtr->getHeight());
    ROS_INFO( "FPS = %d", cameraPtr->getFPS());
}


CameraPublisher::~CameraPublisher()
{
    cameraPtr->close();
}


void CameraPublisher::startStreaming()
{
    std::cout << "Streaming camera images" << std::endl;
    cameraPtr->startTransfer();
    // cameraPtr->join;
}


void CameraPublisher::stopStreaming()
{
    std::cout << "Stop images stream" << std::endl;
    cameraPtr->stopTransfer();
}


void CameraPublisher::receivedFrameCallback(teledyne::Frame frame, void *params)
{
    CameraPublisher *pThis = (CameraPublisher *)params;
    pThis->receivedFrame(&frame);
}


void CameraPublisher::receivedFrame(teledyne::Frame *frame)
{
    if (frame->status == GEVLIB_OK)
    {
        cv::Mat bayer_frame = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC1, &frame->data[0]);
        // Convert frame
        cv::Mat BGR_frame;
        cv::cvtColor(bayer_frame, BGR_frame, CV_BayerBG2BGR);
        // Resize frame
        cv::Size image_size(frame->width, frame->height);
        cv::Mat resized_frame;
        cv::resize(BGR_frame, resized_frame, image_size);

//        saveFrame(resized_frame);
        showFrames(resized_frame);
        publishFrame(resized_frame);
    }
}


void CameraPublisher::saveFrame(const cv::Mat &frame)
{
    std::ostringstream s;
    time_t t = time(0);
    s << "imagem_" << t << ".jpg";
    std::string imageName(s.str());
    cv::imwrite(imageName.c_str(), frame);
}


void CameraPublisher::showFrames(const cv::Mat &frame)
{
    cv::imshow("Live", frame);
    cv::waitKey(1);    
}


void CameraPublisher::publishFrame(const cv::Mat &frame)
{
    sensor_msgs::ImagePtr image = cv_bridge::CvImage(
                                        std_msgs::Header(),
                                        "bgr8",
                                        frame).toImageMsg();
    image->header.stamp = ros::Time::now();
    image->header.frame_id = "camera";
    ROS_INFO_STREAM("Time is: " << frame.cols);

    // ROS_INFO_STREAM("Time is: " << image->header.stamp.sec);
    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(
                cameraInfoPtr->getCameraInfo()));
    cam_info->header.stamp = image->header.stamp;
    cam_info->header.frame_id = image->header.frame_id;
    imagePublisher.publish(image, cam_info);
}
