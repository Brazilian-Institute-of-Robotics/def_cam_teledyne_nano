#include <string>
#include <unistd.h>
#include <ros/ros.h>
#include <application.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, std::string("def_teledyne_node"));
    int delay;
    if(argc<=1) delay = 0; 
    else delay = std::atoi(argv[1]);
    sleep(delay); 
    std::string camera_username = "S1112960";
    App app;
    app.run(camera_username);
    app.startStream();
    return 0;
}
