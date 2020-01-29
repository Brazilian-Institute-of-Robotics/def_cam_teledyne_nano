#include "CameraPublisher.hpp"


using namespace std;


template <typename T>
T getParameter(const ros::NodeHandle &nh,
               const string &param_name)
{
    T param;
    if(nh.hasParam(param_name))
    {
        nh.getParam(param_name, param);
        cout << "Teledyne publisher node parameter: "
             << param_name << " = " << param << endl;
    }else{
        cout << "Error!!! Parameter not defined: " << param_name << endl;
//        ROS_FATAL("Error!!! Parameter not defined: " << param_name);
        ros::shutdown();
        exit(1);
    }
    return param;
}



int main(int argc, char **argv)
{
    unsigned delay = (argc == 2) ? atoi(argv[1]) : 0;
    sleep(delay);

    ros::init(argc, argv, string("publisher"));
    ros::NodeHandle nh("~");

    string node_namespace = getParameter<string>(nh, "node_namespace");
    string camera_serial_number = getParameter<string>(nh, "camera_serial_number");
    string path_to_config_file = getParameter<string>(nh, "path_to_config_file");
    string topic_name = getParameter<string>(nh, "topic_name");
    bool flip_cam = getParameter<bool>(nh, "flip_cam");


    CameraPublisher camera_publisher(nh, camera_serial_number,
                                     path_to_config_file, topic_name, flip_cam);
    camera_publisher.startStreaming();

    ros::spin();
    return 0;
}
