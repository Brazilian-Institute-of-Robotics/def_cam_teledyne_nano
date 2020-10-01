/**
* @file CameraAPI.cpp
* @brief This source contains the camera (Teledyne Genie Nano C2590)
* connections layers, methods and parameters to acquire images.
*
* Available to set up all XML features, camera connections, enable some
* features available and retrieve some important parameters to acquire images.
*
*
* @author Matheus Nascimento (@matheusns).
* @ date July 2017
*/
#include "CameraAPI.hpp"


namespace teledyne
{

//Constructor
Camera::Camera(
    UINT32 width,
    UINT32 height,
    UINT32 format)
    : _width(width)
    , _height(height)
    , _format(format)
    , _handle(NULL)
    , _grabCallback(NULL)
    , _grabCallbackParams(NULL)
    , _threadId(0)
    , _isRunning(false)
    , _isTransfering(false)
    , _snapshot(false)
    , _mutex(PTHREAD_MUTEX_INITIALIZER)
    , _cond(PTHREAD_COND_INITIALIZER)
{
}

Camera::~Camera()
{
}

void Camera::open(const std::string &cameraName)
{
    GEVLIB_CONFIG_OPTIONS options;                 ///< Camera connections options object
    GEV_DEVICE_INTERFACE pCamera[MAX_CAMERAS];     ///< Struct that contains the camera's informations
    GEV_STATUS status = 0;

    GevGetLibraryConfigOptions(&options);
    options.logLevel = GEV_LOG_LEVEL_NORMAL;       ///< Log Messages Level
    GevSetLibraryConfigOptions(&options);          ///< Set default options for the library.

    int numCamera = 0;
    if (GevGetCameraList(pCamera, MAX_CAMERAS, &numCamera) == GEVLIB_OK)
    {
        int cameraIndex = -1;
        for (int i = 0; i < numCamera; i++)
        {
            if (!cameraName.compare(std::string(pCamera[i].username)))
            {
                cameraIndex = i;
                break;
            }
        }

        if (numCamera == 0 || cameraIndex == -1)
        {
            throw Exception("Invalid Camera name = " + cameraName + ", check your connections");
        }

        if ((status = GevOpenCamera(&pCamera[cameraIndex], GevExclusiveMode, &_handle)) != GEVLIB_OK)
        {
            throw Exception(std::string("Camera " + cameraName + "-There isn't a camera that corresponds to this index. Error(" + std::to_string((short int)status) + ")"));
        }

        if (GevInitGenICamXMLFeatures(_handle, TRUE) == GEVLIB_OK)
        {
            char xmlFileName[MAX_PATH] = {0};
            if (GevGetGenICamXML_FileName(_handle, (int)sizeof(xmlFileName), xmlFileName) == GEVLIB_OK)
            {
                GevSetFeatureValueAsString(_handle, "multipleROIMode", "Off");

                if ((status = GevSetImageParameters(_handle, 2592, 2048, 0, 0, _format)) != GEVLIB_OK)
                {
                    throw Exception(std::string("GevSetImageParameters Error: " + std::to_string(status)));
                }

                tuneStreaming();
                prepareBuffer();

                if ((status = GevInitImageTransfer(_handle, Asynchronous, NUM_BUF, _bufAddress)) != GEV_FRAME_STATUS_RECVD)
                {
                    throw Exception(std::string("Init image transfer Error " + std::to_string((short int)status)));
                }
                if ((int)getFPS() == 0)
                {
                    throw Exception(std::string("Init Error " + std::to_string((short int)status)));
                }
                return;
            }
        }

        throw Exception(std::string("Found Error During XML Connection: Erro "));
    }
}

void Camera::tuneStreaming()
{
    GEV_CAMERA_OPTIONS camOptions; ///< Camera options object
    // Adjust the camera interface options
    GevGetCameraInterfaceOptions(_handle, &camOptions);
    camOptions.streamMemoryLimitMax = 64 * 2592 * 2048; // Adjust packet memory buffering limit.
    camOptions.streamFrame_timeout_ms = 1001;           // Internal timeout for frame reception.
    GevSetCameraInterfaceOptions(_handle, &camOptions);
}

void Camera::setFPS(float fps)
{
    GevSetFeatureValue(_handle, "AcquisitionFrameRate", sizeof(float), &fps);
}

void Camera::setWidth(int width)
{
    GevSetFeatureValue(_handle, "Width", sizeof(UINT32), &width);
}

void Camera::setHeight(int height)
{
    GevSetFeatureValue(_handle, "Height", sizeof(UINT32), &height);
}

void Camera::multipleRoiMode(std::string mode)
{
    if(mode =="Active")
    {    
        GevSetFeatureValueAsString(_handle, "multipleROIMode", mode.c_str());
    }
    else
    {
        GevSetFeatureValueAsString(_handle, "multipleROIMode", "Off");
    }
}

void Camera::multipleRoiSelector(std::string region)
{
    GevSetFeatureValueAsString(_handle, "multipleROISelector", region.c_str());
}

void Camera::setRoiCountHorizontal(int value) 
{
    GevSetFeatureValue(_handle, "multipleROICountHorizontal", sizeof(UINT32), &value);
}

void Camera::setRoiCountVertical(int value) 
{
    GevSetFeatureValue(_handle, "multipleROICountVertical", sizeof(UINT32), &value);
}

void Camera::roiCount(int value) 
{
    GevSetFeatureValue(_handle, "multipleROICount", sizeof(UINT32), &value);
}

void Camera::setRoiOffsetX(int value) 
{
    GevSetFeatureValue(_handle, "multipleROIOffsetX", sizeof(UINT32), &value);
}

void Camera::setRoiOffsetY(int value)
{
    GevSetFeatureValue(_handle, "multipleROIOffsetY", sizeof(UINT32), &value);
}

void Camera::setRoiWidth(int value)
{
    GevSetFeatureValue(_handle, "multipleROIWidth", sizeof(UINT32), &value);
}

void Camera::setRoiHeight(int value)
{
    GevSetFeatureValue(_handle, "multipleROIHeight", sizeof(UINT32), &value);
}

void Camera::enableAutoWhiteBalance()
{
    GevSetFeatureValueAsString(_handle, "BalanceWhiteAuto", "OnDemand");
}

void Camera::applyAutoWhiteBalance()
{
    GevSetFeatureValueAsString(_handle, "balanceWhiteAutoOnDemandCmd", "True");
}

void Camera::setExposureMode(std::string mode)
{
    GevSetFeatureValueAsString(_handle, "ExposureMode", mode.c_str());
}

void Camera::setExposureTime(double value)
{
    GevSetFeatureValue(_handle, "ExposureTime", sizeof(double), &value);
}

void Camera::setExposureAlignment(std::string align)
{
    GevSetFeatureValueAsString(_handle, "exposureAlignment", align.c_str());
}

void Camera::setGainMode(std::string option)
{
    GevSetFeatureValueAsString(_handle, "GainSelector", option.c_str());
}

void Camera::setGainRaw(int value)
{
    GevSetFeatureValue(_handle, "GainRaw", sizeof(int), &value);
}

void Camera::setGain(float value)
{
    GevSetFeatureValue(_handle, "Gain", sizeof(float), &value);
}

void Camera::setAutoBrightness(std::string option)
{
    GevSetFeatureValueAsString(_handle, "autoBrightnessMode", option.c_str());
}

void Camera::setAutoBrightnessSequence(std::string option)
{
    GevSetFeatureValueAsString(_handle, "autoBrightnessSequence", option.c_str());
}

void Camera::setAutoBrightnessTarget(int value)
{
    GevSetFeatureValue(_handle, "autoBrightnessTarget", sizeof(int), &value);
}

void Camera::setAutoBrightnessTargetRangeVariation(int value)
{
    GevSetFeatureValue(_handle, "setAutoBrightnessTargetRangeVariation", sizeof(int), &value);
}

void Camera::setAutoBrightnessMinTimeAct(double value)
{
    GevSetFeatureValue(_handle, "autoBrightnessAlgoMinTimeActivation", sizeof(double), &value);
}

void Camera::setAutoBrightnessMaxTimeAct(double value)
{
    GevSetFeatureValue(_handle, "autoBrightnessAlgoConvergenceTime", sizeof(double), &value);
}

void Camera::setAutoExposureMode(std::string option)
{
    GevSetFeatureValueAsString(_handle, "ExposureAuto", option.c_str());
}

void Camera::setAutoExposureMinValue(double value)
{
    GevSetFeatureValue(_handle, "exposureAutoMinValue", sizeof(double), &value);
}    

void Camera::setAutoExposureMaxValue(double value)
{
    GevSetFeatureValue(_handle, "exposureAutoMaxValue", sizeof(double), &value);
}    

void Camera::setAutoGainMode(std::string option)
{
    GevSetFeatureValueAsString(_handle, "GainAuto", option.c_str());
}

void Camera::setAutoGainMaxValue(double value)
{
    GevSetFeatureValue(_handle, "gainAutoMaxValue", sizeof(double), &value);
}

void Camera::setAutoGainMinValue(double value)
{
    GevSetFeatureValue(_handle, "gainAutoMinValue", sizeof(double), &value);
}

std::string Camera::getPixelFormat()
{
    int type;
    char mode[100]={0};
    GevGetFeatureValueAsString(_handle, "PixelFormat", &type, sizeof(mode), mode);
    std::string response(mode);
    return response;
} 

double Camera::getFPS()
{
    int type;
    double fps;
    GevGetFeatureValue(_handle, "AcquisitionFrameRate", &type, sizeof(double), &fps);
    return fps;
}

float Camera::getTemperature()
{
    int type;
    float temperature;
    GevGetFeatureValue(_handle, "DeviceTemperature", &type, sizeof(float), &temperature);
    return temperature;
}

std::string Camera::getExposureMode()
{
    int type;
    char mode[60]={0};
    GevGetFeatureValueAsString(_handle, "ExposureMode", &type, sizeof(mode), mode);
    std::string response(mode);
    return response;
}
double Camera::getExposureTime()
{
    int type;
    double tmp;
    GevGetFeatureValue(_handle, "exposureTimeActual", &type, sizeof(double), &tmp);
    return tmp;
}
std::string Camera::getFirmware()
{
    int type;
    char mode[60]={0};
    GevGetFeatureValueAsString(_handle, "DeviceFirmwareVersion", &type, sizeof(mode), mode);
    std::string response(mode);
    return response;
}
std::string Camera::getGainMode()
{
    int type;
    char mode[60]={0};
    GevGetFeatureValueAsString(_handle, "GainSelector", &type, sizeof(mode), mode);
    std::string response(mode);
    return response;
}
double Camera::getGain()
{
    int type;
    double tmp;
    GevGetFeatureValue(_handle, "Gain", &type, sizeof(double), &tmp);
    return tmp;
}
int Camera::getGainRaw()
{
    int type;
    int tmp;
    GevGetFeatureValue(_handle, "GainRaw", &type, sizeof(int), &tmp);
    return tmp;
}
std::string Camera::getAutoBrightnessMode()
{
    int type;
    char mode[30]={0};
    GevGetFeatureValueAsString(_handle, "autoBrightnessMode", &type, sizeof(mode), mode);
    std::string response(mode);
    return response;
}
int Camera::getAutoBrightnessTarget()
{
    int type;
    int tmp;
    GevGetFeatureValue(_handle, "autoBrightnessTarget", &type, sizeof(int), &tmp);
    return tmp;
}
bool Camera::getAutoExposureMode()
{
    int type;
    char mode[50]={0};
    GevGetFeatureValueAsString(_handle, "ExposureAuto", &type, sizeof(mode), mode);
    std::string response(mode);
    if(response == "Continuous") return true;
    return false;
}

bool Camera::getAutoGainMode()
{
    int type;
    char mode[60]={0};
    GevGetFeatureValueAsString(_handle, "GainAuto", &type, sizeof(mode), mode);
    std::string response(mode);
    if(response == "Continuous") return true;
    return false;
}

bool Camera::enableTurboMode()
{
    int type;
    UINT32 val = 0;
    GevGetFeatureValue(_handle, "transferTurboMode", &type, sizeof(UINT32), &val);
    if (val == 0)
    {
        val = 1;
        GevSetFeatureValue(_handle, "transferTurboMode", sizeof(UINT32), &val);
    }
    GevGetFeatureValue(_handle, "transferTurboMode", &type, sizeof(UINT32), &val);
    if (val == 1)
        return true;
    else
        return false;
}
bool Camera::disableTurboMode()
{
    int type;
    UINT32 val = 1;
    GevGetFeatureValue(_handle, "transferTurboMode", &type, sizeof(UINT32), &val);
    if (val == 1)
    {
        val = 0;
        GevSetFeatureValue(_handle, "transferTurboMode", sizeof(UINT32), &val);
        GevGetFeatureValue(_handle, "transferTurboMode", &type, sizeof(UINT32), &val);
        if (val == 0)
            return true;
        else
            return false;
    }
    else
        return true;
}

void Camera::close()
{
    if (_isRunning)
    {
        _isRunning = false;
        pthread_join(_threadId, NULL);
    }

    if (_isTransfering)
    {
        stopTransfer();
    }
    GevAbortImageTransfer(_handle);
    GevFreeImageTransfer(_handle);
    releaseBuffer();
    GevCloseCamera(&_handle);
}

void Camera::imageToFrame(GEV_BUFFER_OBJECT *image, Frame &frame)
{
    frame.width = image->w;
    frame.height = image->h;
    frame.status = image->status;
    frame.depth = GetPixelSizeInBytes(_format);
    frame.data.resize(image->recv_size);
    memcpy(&frame.data[0], image->address, image->recv_size);
}

void Camera::prepareBuffer()
{
    int depth = GetPixelSizeInBytes(_format);
    int size = _width * _height * depth;
    for (int i = 0; i < NUM_BUF; i++)
    {
        _bufAddress[i] = (PUINT8)malloc(size);
        memset(_bufAddress[i], 0, size);
    }
}

void Camera::releaseBuffer()
{
    for (int i = 0; i < NUM_BUF; i++)
    {
        if (_bufAddress[i])
        {
            free(_bufAddress[i]);
            _bufAddress[i] = NULL;
        }
    }
}

void Camera::cleanBuffer()
{
    int depth = GetPixelSizeInBytes(_format);
    int size = _width * _height * depth;
    for (int i = 0; i < NUM_BUF; i++)
    {
        memset(_bufAddress[i], 0, size);
    }
}

void Camera::startTransfer()
{
    if (!_isRunning)
    {
        throw Exception(std::string("Should call setupCallback function first."));
    }

    GEV_STATUS status = 0;
    if (!_isTransfering)
    {
        if ((status = GevStartImageTransfer(_handle, -1)) != GEVLIB_OK)
        {
            throw Exception(std::string("Start transfer error:  " + std::to_string((short int)status)));
        }
        _isTransfering = true;

        // GEVLIB_ERROR_NO_CAMERA
    }
}

void Camera::stopTransfer()
{
    if (_isTransfering)
    {
        GEV_STATUS status = 0;
        _isTransfering = false;
        if ((status = GevStopImageTransfer(_handle) != GEVLIB_OK))
        {
            throw Exception(std::string("Stop Transfer Error " + std::to_string((short int)status)));
        }
    }
}

Frame Camera::snapshot(bool& frameAcquired)
{
    _snapshot = true;
    frameAcquired = false;
    Frame ret;
    GEV_STATUS status;
    int frameStatus;

    struct timespec timeout;
    struct timeval now;
    gettimeofday(&now,NULL);
    timeout.tv_sec =now.tv_sec+10;
    timeout.tv_nsec = (now.tv_usec+1000UL*10)*1000UL;

    startTransfer();    
    pthread_mutex_lock(&_mutex);
    frameStatus = pthread_cond_timedwait(&_cond, &_mutex, &timeout);
    if(frameStatus == 0) frameAcquired = true;
    std::cout << "Mutex Cond value = " << frameStatus << std::endl;
    _snapshot = false;
    ret = _frame;
    pthread_mutex_unlock(&_mutex);
    stopTransfer();
    return ret;
}

void Camera::setupCallback(GRABCALLBACK callback, void *params)
{
    if (callback == NULL)
    {
        throw Exception(std::string("the parameter 'callback' should not be NULL."));
    }

    if (!_isRunning)
    {
        _isRunning = true;
        _grabCallback = callback;
        _grabCallbackParams = params;
        pthread_create(&_threadId, NULL, imageReceiveThread, this);
    }
}

const std::string Camera::outputFrameError(int frame_error) {
  switch (frame_error) {
    case GEV_FRAME_STATUS_PENDING:
      return "Frame not ready yet";
    case GEV_FRAME_STATUS_TIMEOUT:
      return "Frame timed out";
    case GEV_FRAME_STATUS_OVERFLOW:
      return "Frame overflow, buffer was full";
    case GEV_FRAME_STATUS_BANDWIDTH:
      return "Frame bandwith not enought, too many resends";
    case GEV_FRAME_STATUS_LOST:
      return "Lost resend operations";
    case GEV_FRAME_STATUS_RELEASED:
      return "Device has been released";
  }
}

void *Camera::imageReceiveThread(void *params)
{
    GEV_STATUS status = 0;
    Frame tmpFrame;
    Camera *pThis = (Camera *)params;
    while (pThis->_isRunning)
    {
        GEV_BUFFER_OBJECT *img = NULL;
        status = GevWaitForNextImage(pThis->_handle, &img, 1000);
        //  std::cout << "Snapshot Wait for the next image Error: "<<(short int)status<<std::endl;
        if (img != NULL && pThis->_grabCallback != NULL)
        {
            if (status == GEVLIB_OK && img->status == GEV_FRAME_STATUS_RECVD)
            {
                pthread_mutex_lock(&pThis->_mutex);
                pThis->imageToFrame(img, pThis->_frame);
                if(pThis->_snapshot && pThis->_frame.width == 2592) pthread_cond_signal(&pThis->_cond);
                pthread_mutex_unlock(&pThis->_mutex);
                if(pThis->_frame.width == 2592 )
                {
                    pThis->_grabCallback(pThis->_frame, pThis->_grabCallbackParams);
                }
                else pThis->_grabCallback(pThis->_frame, pThis->_grabCallbackParams);
                if(pThis->_frame.width == 1024)
                {
                    tmpFrame = pThis->_frame;
                }
            } else {
              
              std::cerr << outputFrameError(img->status) << std::endl;
            }
        }
    }
    pthread_exit(0);
    return NULL;
}

void Camera::join()
{
    pthread_join(_threadId, NULL);
}

int Camera::cameraStatus(std::string userName)
{
    GEV_CAMERA_HANDLE handle = NULL;
    GEV_STATUS status = GevOpenCameraByName((char *)userName.c_str(), GevMonitorMode, &handle);

    // printf("Camera Status = 0x%08x\n", status);
    if (status == GEV_STATUS_SUCCESS)
    {
        GevCloseCamera(&handle);
        return CAMERA_AVAILABLE;
    }
    else
    {
        if (status == GEV_STATUS_ACCESS_DENIED || status == GEV_STATUS_NO_MSG)
        {
            return CAMERA_CONNECTED;
        }
        else
        {
            return CAMERA_UNAVAILABLE;
        }
    }
}

} // namespace teledyne
