#include <application.h>
App::App()
  : camera(2592, 2048, fMtBayerRG8)
{
}

App::~App()
{
  camera.close();
}

void App::run(std::string camera_username)
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

void App::receivedFrame(teledyne::Frame* frame)
{

  if (frame->status == GEVLIB_OK)
  {
      cv::Mat mat = cv::Mat(cv::Size(frame->width, frame->height), CV_8UC1, &frame->data[0]);
      cv::Mat bgr;
      cv::cvtColor(mat, bgr, CV_BayerBG2BGR);
      std::string title = "NICEEEEEEE";
      cv::namedWindow(title.c_str(), CV_WINDOW_NORMAL);
      cv::imshow(title.c_str(), bgr);
      cv::waitKey(25);
  }

}

void App::saveFrame(cv::Mat mat) {
    std::ostringstream s;
    time_t t = time(0);
    s << "imagem_" << t << ".jpg";
    std::string imageName(s.str());
    cv::imwrite(imageName.c_str(), mat);
}

void App::startStream()
{
  camera.startTransfer();
  camera.join();
}

void App::receivedFrameCallback(teledyne::Frame frame, void *params) {
    App *pThis = (App*)params;
    pThis->receivedFrame(&frame);
}

