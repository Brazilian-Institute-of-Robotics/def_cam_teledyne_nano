/**
* @file camera.hpp
* @brief Function prototypes for the Camera driver.
*
* This file contains the prototypes for the Camera
* driver and eventually its macros, constants,
* and methods, necessary to interacting with other applications.
*
* @author Matheus Nascimento (@matheusns).
* @ date October 2017
*/

#ifndef DATHOMIR_INCLUDE_CAMERA_H
#define DATHOMIR_INCLUDE_CAMERA_H

#include <exception>
#include <iostream>
#include <ostream>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>

#include <opencv2/opencv.hpp>               
#include <opencv2/core/core.hpp>            
#include <opencv2/highgui/highgui.hpp>      

#include "cordef.h"             
#include "GenApi/GenApi.h"		
#include "gevapi.h"				

#define NUM_BUF	8

namespace teledyne
{
/*! dathomir::enum that is used to indicate the camera current status. */
enum enumStatus
{
    CAMERA_CONNECTED,       ///< Camera connected 
    CAMERA_AVAILABLE,       ///< Camera available to connection
    CAMERA_UNAVAILABLE      ///< Camera unavailable or disconnected
};
/*! \struct Frame camera.hpp "include/camera.hpp"
 *  \brief Struct that contains all necessary images features.
 */
struct Frame {
    Frame() 
    : time(0)       
    , width(0)      
    , height(0)     
    , depth(0)      
    , status(1)      
    {}
    /*! Parameterized constructor. */
    Frame(time_t time, int width, int height, int depth, int status)
    {
        data.resize(width * height * depth);
    }

    time_t time;                    ///< Image timestamp
    int width;                      ///< Image width
    int height;                     ///< Image height
    int depth;                      ///< Image depth
    int status;                     ///< Image status
    std::vector<uchar> data;        ///< Vector data 
};

class Exception : public std::exception {
public:
    /**
    *   @brief  Default constructor for Exception.
    *
    *   @param msg Thrown Exception message.
    *   
    *   @return nothing.
    */
    Exception(std::string msg) {
        _msg = msg;
    }
    /**
    *   @brief  Default destructor for Exception.
    *
    *   @return nothing.
    */
    virtual ~Exception() throw() {
    }
    /**
    *   @brief  Overloaded method from std::Exception that shows up the error messages.
    *
    *   @return const char*.
    */
    virtual const char* what() const throw() {
        return _msg.c_str();
    }
protected:
    std::string _msg;   ///< Exception message.  
};

/*! \typedef  GRABCALLBACK function pointer.
* Defines a function pointer that receives this types of parameters.
*/
typedef void (*GRABCALLBACK) (Frame frame, void*);

class Camera {
public:
    /**
    *   @brief  Parameterized constructor for Camera.
    *
    *   @param width Frame width.
    *   @param height Frame heigth.
    *   @param pixelFormat Frame pixel format.
    *   
    *   @return nothing.
    */
    Camera(UINT32 width, UINT32 height, UINT32 pixelFormat);
    /**
    *   @brief  Default destructor for Camera.
    *   @return nothing.
    */
    ~Camera();
    /** @brief Tunes some acquisition parameters.
    *
    *   Tunes some acquisition parameters as the following:
    *   Internal timeout for frame reception,
    *   Buffer frames internally,
    *   Adjust packet memory buffering limit,
    *   GVSP packet size,
    *   usecs between packets to pace arrival at NIC.
    *
    *   @return void.
    */
    void tuneStreaming();
    /** @brief Sets the frame width.
    *
    *   Sets the frame width (64 - 2592) accessing the camera XML features.
    *
    *   @param width Frame width desired.
    *   @return void.
    */
    void setWidth(int width);
    /** @brief Sets the frame height.
    *
    *   Sets the frame width (64 - 2048) accessing the camera XML features.
    *
    *   @param width Frame height desired.
    *   @return void.
    */
    void setHeight(int height);
    /** @brief Enables/Disables the multiple ROI mode.
    *
    *   This method enables/disables the multiple ROI (Region Of Interest) mode.
    *
    *   @param mode Desired mode.
    *   @return void.
    */
    void multipleRoiMode(std::string mode);
    /** @brief Selects the desired ROI.
    *
    *   This method selects the desired ROI.
    *
    *   @param mode Desired mode.
    *   @return void.
    */
    void multipleRoiSelector(std::string region);
    /** @brief Sets the horizontal ROI quantity.
    *
    *   Sets the horizontal ROI quantity.
    *
    *   @param value Horizontal ROI quantity.
    *   @return void.
    */
    void setRoiCountHorizontal(int value);
    /** @brief Sets the vertical ROI quantity.
    *
    *   Sets the vertical ROI quantity.
    *
    *   @param value Vertical ROI quantity.
    *   @return void.
    */
    void setRoiCountVertical(int value); 
    /** @brief Sets ROI quantity.
    *
    *   Sets the ROI quantity.
    *
    *   @param value ROI quantity.
    *   @return void.
    */
    void roiCount(int value); 
    /** @brief Sets the horizontal ROI offset.
    *
    *   Sets the horizontal ROI offset.
    *
    *   @param value Horizontal ROI offset value.
    *   @return void.
    */
    void setRoiOffsetX(int offsetX);
    /** @brief Sets the vertical ROI offset.
    *
    *   Sets the vertical ROI offset.
    *
    *   @param value ROI Vertical offset value.
    *   @return void.
    */
    void setRoiOffsetY(int offsetY);
    /** @brief Sets the ROI height.
    *
    *   Sets the ROI height.
    *
    *   @param roiHeight ROI height.
    *   @return void.
    */
    void setRoiHeight(int roiHeight);
    /** @brief Sets the ROI width.
    *
    *   Sets the ROI width.
    *
    *   @param roiHeight ROI width.
    *   @return void.
    */
    void setRoiWidth(int roiWidth);
    /** @brief Sets the frame rate.
    *
    *   Sets the frame rate (frames per seconds) accessing the camera XML features.
    *
    *   @param height Frame rate desired.
    *   @return void.
    */
    void setFPS(float fps);
    /** @brief Returns the current frame width.
    *
    *   Returns the current frame width.
    *
    *   @return int.
    */
    UINT32 getWidth() const { return _width; }
    /** @brief Returns the current frame height.
    *
    *   Returns the current frame height.
    *
    *   @return int.
    */
    UINT32 getHeight() const { return _height; }
    /** @brief Returns the current pixel format.
    *
    *   Returns the camera current pixel format.
    *   @return std::string.
    */
    std::string getPixelFormat();
    /** @brief Returns the current frame rate.
    *
    *   Returns the current frame rate.
    *
    *   @return float.
    */
    float getFPS();
    /** @brief Returns the temperature of the current device in degrees Celsius.
    *
    *   Returns the temperature of the current device in degrees Celsius. Maximum temperature should not 
    *   exceed +70ºC for reliable operation. 
    *
    *   @return float.
    */
    float getTemperature();
    /** @brief Enables the camera turbo mode.
    *
    *   If the Camera's firmware has a turbo mode support this function enables it.
    *
    *   @return bool.
    */
    bool enableTurboMode();
    /** @brief Disables the camera turbo mode.
    *
    *   If the Camera's firmware has a turbo mode support and it has been activated,
    *   this function disables it.
    *
    *   @return bool.
    */
    bool disableTurboMode();
    /** @brief Initializes the Cameras connections.
    *
    *   Uses the GevAPI to initialize the connections with the desired camera through camera user name.\
    *   The camera options also are configured with this method.
    *
    *   @param cameraName The camera user name that is used to do the connections.
    *
    *   @return void.
    */
    void open(const std::string& cameraName);
    /** @brief Initializes the necessary components to start the image acquisition.
    *
    *   Initializes the responsible thread that starts the image acquisition.\ Checks
    *   the function pointer that receives the images. 
    *
    *   @param callback Function pointer of GRABCALLBACK type.
    *   @param params Void pointer that is used to access the passed object by reference.
    *   @return void.
    */
    void setupCallback(GRABCALLBACK callback, void *params);
    /** @brief  Closes all camera data transfer initialized.
    *
    *   This method closes the camera data transfer and cleans all allocated memory.    
    *
    *   @return void.
    */
    void close();
    /** @brief Starts the continuous acquisition.
    *
    *   This method starts the camera continuous acquisition.\ Once initialized, 
    *   this method must be stopped before a new transfer.
    *
    *   @return void.
    */
    void startTransfer();
    /** @brief Stops the current transfer.
    *
    *   This method stops the current transfer.\
    *   This method must be called after each snapshot and after each continuous transfer.   
    *
    *   @return void.
    */
    void stopTransfer();
    /** @brief Returns the most recently frame acquired.
    *
    *   This method returns the most recently frame acquired.\
    *   If the continuous transfer is enabled, it must be stopped before executing this method.
    *
    *   @param frameAcquired Frame acquired status.
    *   @return dathomir::Frame.
    */
    Frame snapshot(bool& frameAcquired);
    /** @brief Returns the current camera status.
    *
    *   This method returns an dathomir's enum integer with the current camera status.\
    *
    *   @return dathomir::enum.
    */
    int cameraStatus(std::string username);
    /** @brief Enables the Auto white balance.
    *
    *   This method enables the feature Auto white balance.\
    *   This feature improves the image quality through the white balance adjustment
    *   in each color channel. 
    *
    *   @return void.
    */
    void enableAutoWhiteBalance();
    /** @brief Applies the Auto white balance.
    *
    *   This method applies the feature Auto white balance .\
    *   This feature improves the image quality through the white balance adjustment
    *   in each color channel.\
    *   To aplly this feature, the enableAutoWhiteBalance function must be called before.  
    *
    *   @return void.
    */
    void applyAutoWhiteBalance();
    /** @brief Waits the _threadId finish.
    *
    *   This method returns when the _threadId execution has completed.
    *
    *   @return void.
    */
    void join();
    /** @brief Sets the camera exposure mode.
    *
    *   Sets the camera exposure mode.
    *
    *   @param mode Desired exposure mode.
    *   @return void.
    */
    void setExposureMode(std::string mode);
    /** @brief Sets the camera exposure time.
    *
    *   Sets the camera exposure mode.
    *
    *   @param value Desired exposure time value.
    *   @return void.
    */
    void setExposureTime(double value);
    /** @brief Sets the camera exposure alignment.
    *
    *   Sets the camera exposure alignment.
    *
    *   @param align Desired exposure alignment.
    *   @return void.
    */
    void setExposureAlignment(std::string align);
    /** @brief Sets the camera gain mode.
    *
    *   Selects which gain is controlled when adjusting gain features.
    *   Possible Values:
    *   'DigitalAll'
    *   'SensorDigital'
    *   'SensorAnalog'
    *
    *   @param option Desired gain option.
    *   @return void.
    */
    void setGainMode(std::string option);
    /** @brief Sets the camera gain value.
    *
    *   Sets the selected gain as an amplification factor applied to the image.
    *
    *   @param value Desired gain value.
    *   @return void.
    */
    void setGain(float value);
    /** @brief Raw Gain value that is set in the camera.
    *
    *   Sets the camera raw gain value.
    *
    *   @param value Desired gain raw.
    *   @return void.
    */
    void setGainRaw(int value);
    /** @brief Set the mode for the Auto-Brightness.
    *
    *   When enable, the Auto-Brightness function adjusts the gain and exposure to adapt changing light conditions.
    *
    *   @param option Desired auto brightness option.
    *   @return void.
    */
    void setAutoBrightness(std::string option);
    /** @brief Sets the camera auto brightness sequence.
    *
    *   Specifies the processing order for the auto-brightness algorithm. Gain and exposure are adjusted sequentially,
    *   in the selected order to achieve the auto-brightness target value.
    *   Possible Values:
    *   'Gain_Exposure_Iris'
    *   'Exposure_Gain_Iris'
    *
    *   @param value Desired exposure alignment.
    *   @return void.
    */
    void setAutoBrightnessSequence(std::string option);
    /** @brief Sets the camera auto brightness target.
    *
    *   Sets the target image grayscale value, in DN, for the auto-brightness algorithm.
    *
    *   @param value Desired target value in DN (Digital number).
    *   @return void.
    */
    void setAutoBrightnessTarget(int value);
    /** @brief Sets the camera auto brightness target variation.
    *
    *   Sets the auto-brightness target range variation (0 - 255), in DN. An auto-brightness target value whitin this range
    *   is considered valid and will not be compensated.  
    *
    *   @param value Desired target variation value in DN (Digital number).
    *   @return void.
    */
    void setAutoBrightnessTargetRangeVariation(int value);
    /** @brief Sets the camera auto brightness minimum time activation.
    *
    *   Specifies the time delay between an image brightness change from the auto-brightness target and when
    *   the compesation exposure/gain starts.
    *
    *   @param value Desired minimum time activation value.
    *   @return void.
    */
    void setAutoBrightnessMinTimeAct(double value);
    /** @brief Sets the camera auto brightness maximum time activation.
    *
    *   Specifies the maximum time the auto-brightness algorithm is allowed to compensate the image brightness 
    *   as defined by the auto-brightness target.
    *
    *   @param value Desired maximum time activation value.
    *   @return void.
    */
    void setAutoBrightnessMaxTimeAct(double value);
    /** @brief Sets the camera auto exposure mode.
    *
    *   Sets the automatic exposure. When Off, exposure duration is manually controled using the exposure feature.
    *
    *   @param option Desired auto exposure mode.
    *   @return void.
    */
    void setAutoExposureMode(std::string option);
    /** @brief Sets the camera auto exposure minimum value.
    *
    *   Sets the minimum automatic exposure time value allowed by the user.
    *
    *   @param value Desired exposure minimum value.
    *   @return void.
    */
    void setAutoExposureMinValue(double value);
    /** @brief Sets the camera auto exposure maximum value.
    *
    *   Sets the maximum automatic exposure time value allowed by the user.
    *
    *   @param value Desired exposure maximum value.
    *   @return void.
    */
    void setAutoExposureMaxValue(double value);
    /** @brief Sets the camera auto gain mode.
    *
    *   Sets the automatic gain. When Off, the selected gain is manually controled using the exposure feature.
    *
    *   @param option Desired auto gain mode.
    *   @return void.
    */
    void setAutoGainMode(std::string option);
    /** @brief Sets the camera auto gain maximum value.
    *
    *   Sets the maximum gain multiplier value for the automatic gain algorithm. The automatic gain function is an
    *   amplification factor apllied to the video signal to obtain the auto-brightness target value.    
    *
    *   @param value Desired exposure maximum value.
    *   @return void.
    */
    void setAutoGainMaxValue(double value);
    /** @brief Sets the camera auto gain minimum value.
    *
    *   Sets the minimum gain multiplier value for the automatic gain algorithm. The automatic gain function is an
    *   amplification factor apllied to the video signal to obtain the auto-brightness target value.    
    *
    *   @param value Desired exposure minimum value.
    *   @return void.
    */
    void setAutoGainMinValue(double value);
    /** @brief Gets the camera exposure mode.
    *
    *   Returns the camera exposure mode.
    *
    *   @return std::string.
    */
    std::string getExposureMode();
    /** @brief Gets the camera exposure time.
    *
    *   Returns the camera exposure time.
    *
    *   @return double.
    */
    double getExposureTime();
    /** @brief Gets the camera gain mode.
    *
    *   Returns the camera gain mode.
    *
    *   @return std::string.
    */
    std::string getGainMode();
    /** @brief Gets the camera firmware version.
    *
    *   Returns the camera firmware version.
    *
    *   @return std::string.
    */
    std::string getFirmware();
    /** @brief Gets the camera gain.
    *
    *   Returns the camera gain.
    *
    *   @return double.
    */
    double getGain();
    /** @brief Gets the camera gain raw.
    *
    *   Returns the camera gain raw.
    *
    *   @return int.
    */
    int getGainRaw();
    /** @brief Gets the camera auto brightness mode.
    *
    *   Returns the camera auto brightness mode.
    *
    *   @return std::string.
    */
    std::string getAutoBrightnessMode();
    /** @brief Gets the camera auto brightness target.
    *
    *   Returns the camera auto brightness target.
    *
    *   @return int.
    */
    int getAutoBrightnessTarget();
    /** @brief Gets the camera auto exposure mode.
    *
    *   Returns the camera auto exposure mode.
    *
    *   @return bool.
    */
    bool getAutoExposureMode();
    /** @brief Gets the camera auto gain mode.
    *
    *   Returns the camera auto gain mode.
    *
    *   @return bool.
    */
    bool getAutoGainMode();
    /** @brief Returns the transfering state.
    *
    *   This method returns the transfering state.
    *
    *   @return void.
    */
    bool isTransfering() { return _isTransfering; }
    /** @brief Prepares the images buffer addresses.
     *
     *   Allocates the required memory according to the size of the images.
     *
     *   @return void.
     */
     void prepareBuffer();
     /** @brief Cleans the buffer addresses.
     *
     *   Cleans the buffer addresses.
     *
     *   @return void.
     */
     void cleanBuffer();
     /** @brief Releases the buffer addresses.
     *
     *   Releases the buffer addresses and assign NULL to each one of them.
     *
     *   @return void.
     */
     void releaseBuffer();

private:
    /** @brief Thread that receives the frames acquired.
    *
    *   This Thread returns, using the _grabCallback function pointer, the frames acquired.
    *
    *   @return void.
    */
    static void* imageReceiveThread(void *params);
    /** @brief Converts the image pointer into the dathomir::frame struct.  
    *
    *   This method Converts the image pointer and its features into the dathomir::frame struct.  
    *
    *   @return void.
    */
    void imageToFrame(GEV_BUFFER_OBJECT *image, Frame& frame);

    GEV_CAMERA_HANDLE _handle;      ///< Camera Handle
    UINT32 _height;                 ///< Frame height
    UINT32 _width;                  ///< Frame weight
    UINT32 _format;                 ///< Frame format
    PUINT8 _bufAddress[NUM_BUF];    ///< Buffer Addresses
    Frame _frame;                   ///< Frame object

    GRABCALLBACK _grabCallback;     ///< Function pointer 
    void *_grabCallbackParams;      ///< Void pointer    
    pthread_t _threadId;            ///< Pthread object   
    pthread_mutex_t _mutex;         ///< Mutex to coordinates the thread's priority   
    pthread_cond_t _cond;           ///< Thread condition

    bool _snapshot;
    bool _isRunning;                ///< Variable to indicates if the class is running 
    bool _isTransfering;            ///< Variable to indicates if the transfering is running    
};

} // namespace dathomir

#endif // DATHOMIR_INCLUDE_CAMERA_H
