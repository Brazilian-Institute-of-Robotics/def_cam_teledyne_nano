#include <application.h>

App::App(ros::NodeHandle nh) : camera(2592, 2048, fMtBayerRG8),
                               nh_(nh),
                               camera_name_("camera1")
{
    cinfo_ = new camera_info_manager::CameraInfoManager(nh_, camera_name_, "package://def_genie_teledyne/config/camera1.yaml");
    it_ = new image_transport::ImageTransport(nh_);
    image_pub_ = it_->advertiseCamera("/dyn_ef_robot/camera1/image_raw", 1);
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
        cv::Size smaller(frame->width/4, frame->height/4);
        cv::Mat temp;
        cv::cvtColor(mat, temp, CV_BayerBG2BGR);
        cv::resize(temp,out,smaller);

        // get image
        sensor_msgs::ImagePtr image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", out).toImageMsg();
        image->header.stamp = ros::Time::now();
        image->header.frame_id = "camera";
        // ROS_INFO_STREAM("Time is: " << image->header.stamp.sec);
        // get current CameraInfo data
        sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        cam_info->header.stamp = image->header.stamp;
        cam_info->header.frame_id = image->header.frame_id;

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
    ros::NodeHandle nh("/dyn_ef_robot/camera1");

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
