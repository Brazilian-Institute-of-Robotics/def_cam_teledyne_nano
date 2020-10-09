#include "CameraPublisher.hpp"


CameraPublisher::CameraPublisher(const ros::NodeHandle &node_handle,
         const std::string &camera_serial_number,
         const std::string &path_to_config,
         const std::string &topic_name,
         const std::string &camera_name)
{
    ros::NodeHandle image_params_handle(node_handle, "image");
    cameraName = camera_name;
    cameraInfoPtr = std::make_shared<camera_info_manager::CameraInfoManager>(
                node_handle,
                camera_name,
                path_to_config);
    readParams(image_params_handle);
    imagePublisher = image_transport::ImageTransport(
                node_handle).advertiseCamera( topic_name, 1 );
    cameraPtr = std::make_shared<teledyne::Camera>(2592, 2048, getTeledynePixelFormat());
    cameraPtr->open(camera_serial_number);
    std::cout << "Opened" << std::endl;
    cameraPtr->setFPS(framePerSecond);
    cameraPtr->setupCallback(CameraPublisher::receivedFrameCallback, this);
    cameraPtr->enableAutoWhiteBalance();
    cameraPtr->applyAutoWhiteBalance();

    ROS_INFO( "Width = %d", cameraPtr->getWidth());
    ROS_INFO( "Height = %d", cameraPtr->getHeight());
    ROS_INFO( "FPS = %d", cameraPtr->getFPS());
}

void CameraPublisher::readParams(const ros::NodeHandle &node_handle) {
  int pixel_format;
  /* TODO Fix image_format parameter */
  // node_handle.param<int>("image_format", pixel_format, 0);

  pixelFormat = static_cast<TeledynePixelFormat>(pixel_format);
  node_handle.param<std::string>("frame_id", frameId, "camera");
  node_handle.param<int>("fps", framePerSecond, 5);
  node_handle.param<int>("width", frameWidth, cameraInfoPtr->getCameraInfo().width);
  node_handle.param<int>("height", frameHeight, cameraInfoPtr->getCameraInfo().height);
  node_handle.param<double>("temperature_threshold", temperatureThreshold, 56);
  if (frameWidth <= 0 || frameHeight <= 0) {
    ROS_WARN("Could not read Height and Width parameters from server, seting default");
    frameWidth = 2592;
    frameHeight = 2048;
  }
}


int CameraPublisher::getTeledynePixelFormat() {
  switch (pixelFormat) {
    case TeledynePixelFormat::BAYER_RG8:
      return fmtBayerRG8;
      break;
    case TeledynePixelFormat::MONO:
      return fmtMono8;
      break;
  }
}

int CameraPublisher::getOpenCVPixelConversion() {
  switch (pixelFormat) {
    case TeledynePixelFormat::BAYER_RG8:
      return CV_BayerBG2BGR;
      break;
    case TeledynePixelFormat::MONO:
      return CV_GRAY2BGR;
      break;
  }
}

std::string CameraPublisher::getRosImagePixelFormat() {
  switch (pixelFormat) {
    case TeledynePixelFormat::BAYER_RG8:
      return "bgr8";
      break;
    case TeledynePixelFormat::MONO:
      return "mono8";
      break;
  }
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
        cv::cvtColor(bayer_frame, BGR_frame, getOpenCVPixelConversion());
        // Resize frame
        cv::Size image_size(frameWidth, frameHeight);
        cv::Mat resized_frame;
        cv::resize(BGR_frame, resized_frame, image_size);
        if (cameraPtr->getTemperature() >= temperatureThreshold) {
          ROS_INFO_STREAM("Camera Termperatures: " << cameraPtr->getTemperature());
        }

//        saveFrame(resized_frame);
        // showFrames(resized_frame);
        publishFrame(resized_frame);
    } else {
      ROS_ERROR_STREAM("Teledyne error: " << frame->status);
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
                                        getRosImagePixelFormat(),
                                        frame).toImageMsg();
    image->header.stamp = ros::Time::now();
    image->header.frame_id = frameId;
    // ROS_INFO_STREAM("Time is: " << frame.cols);

    // ROS_INFO_STREAM("Time is: " << image->header.stamp.sec);
    // get current CameraInfo data
    sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(
                cameraInfoPtr->getCameraInfo()));
    cam_info->header.stamp = image->header.stamp;
    cam_info->header.frame_id = image->header.frame_id;
    imagePublisher.publish(image, cam_info);
}
