# def_cam_teledyne_nano

Dynamic End-effector. Repository to implement RGB cam (Teledyne Nano) on arm manipulator.
The implemented node takes the image from the camera, scales it to 648x512 and publishes over ros image_transport.

## Installation

You need to download and install the DALSA framework for Ubuntu first. You need to sign up on their website to download it, just use trash-data for it.
Follow the given installation steps.

## Prerequisites

A local link network will be used to communicate with the camera, so you need to set it up first. The network has to be configured to support jumbo frames in order to cope
with the huge amount of data. To do this, the DALSA framework provides a script, just run ```sudo GigE-V-Framework_2.02.0.0132/DALSA/GigeV/bin/gev_nettweak <network_interface_which_connects_camera>```. When the network interfaces are rebooted, the script needs to be run again.

## How to use?

Just start a ```roscore``` or launch some launch file, then ```rosrun def_genie_teledyne camera_example```.
The folder ```config/``` contains two camera calibration files for the lens EDMUND OPTICS 16mm/f1.4, one for the downscaled resolution, one for the original
of 2592x2048.