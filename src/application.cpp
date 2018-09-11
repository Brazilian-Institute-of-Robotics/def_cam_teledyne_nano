#include <application.h>

App::App(ros::NodeHandle nh) : camera(2592, 2048, fMtBayerRG8),
                               nh_(nh),
                               camera_name_("camera"),
                               cinfo_(new camera_info_manager::CameraInfoManager(nh_)),
                               it_(new image_transport::ImageTransport(nh_)),
                               image_pub_(it_->advertiseCamera("/dyn_ef_robot/camera1/image_raw", 1))
{
    // set camera name for the camera_info_manager
    if(!cinfo_->setCameraName(camera_name_))
    {
        ROS_WARN_STREAM("[" << camera_name_ << "] name not valid for camera_info_manger");
    }
}

App::~App()
{
    camera.close();
}

void App::run(std::string camera_username, int argc, char **argv)
{
    sleep(2);
    camera.open(camera_username);
    std::cout << "Opened" << std::endl;
    camera.setFPS(20);
    camera.setupCallback(App::receivedFrameCallback, this);
    camera.enableAutoWhiteBalance();
    camera.applyAutoWhiteBalance();

    std::cout << "Width = " << camera.getWidth() << std::endl;
    std::cout << "Height = " << camera.getHeight() << std::endl;
    std::cout << "FPS =  " << camera.getFPS() << std::endl;
}

void App::receivedFrame(teledyne::Frame *frame)
{

    if (frame->status == GEVLIB_OK)
    {
        cv::Mat mat = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC1, &frame->data[0]);
        cv::cvtColor(mat, out, CV_BayerBG2BGR);

        // get image
        sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
        // get current CameraInfo data
        sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

        // publish camera info
        image_pub_.publish(image, cam_info);
    }
}

void App::saveFrame(cv::Mat mat)
{
    std::ostringstream s;
    time_t t = time(0);
    s << "imagem_" << t << ".jpg";
    std::string imageName(s.str());
    cv::imwrite(imageName.c_str(), mat);
}

void App::startStream()
{
    camera.startTransfer();
    // camera.join;
}

void App::receivedFrameCallback(teledyne::Frame frame, void *params)
{
    App *pThis = (App *)params;
    pThis->receivedFrame(&frame);
}

cv::Mat *App::getCurrentImage()
{
    return &out;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, std::string("def_teledyne_node"));
    ros::NodeHandle nh;

    int delay;
    if (argc <= 1)
        delay = 0;
    else
        delay = std::atoi(argv[1]);
    sleep(delay);

    std::string camera_username = "S1112960";
    App app(nh);
    app.run(camera_username, argc, argv);
    app.startStream();
    ros::spin();
    return 0;
}
