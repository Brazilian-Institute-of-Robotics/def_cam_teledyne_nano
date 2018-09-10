#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include <application.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
    ROS_INFO("hi, hier bin ich");
    ros::init(argc, argv, std::string("def_teledyne_node"));
    ros::NodeHandle nh;
    ROS_INFO("node handle initialisiert");

    int delay;
    if (argc <= 1)
        delay = 0;
    else
        delay = std::atoi(argv[1]);
    sleep(delay);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/dyn_ef_robot/camera1/image_raw", 1);

    ROS_INFO("imagetransport");

    std::string camera_username = "S1112960";
    App app;
    // app.run(camera_username);
    // app.startStream();
    cv::Mat *pic;
    ROS_INFO("anfangepenis");

    ros::Rate loop_rate(5);
    while (nh.ok())
    {
        ROS_INFO("penis");
        pic = app.getCurrentImage();
        // pic = cv::imread("/home/simon/Downloads/bild.png", CV_LOAD_IMAGE_COLOR);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *pic).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
