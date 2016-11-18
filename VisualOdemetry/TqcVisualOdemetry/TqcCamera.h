#ifndef TQCCAMERA_H
#define TQCCAMERA_H

#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>

using namespace cv;

int GetCameraDeviceCount();

class CCamera
{
    typedef enum _CameraMode
    {
        Single_Camera_Mode,
        Stereo_Camera_Mode,
        RGBD_Camera_Mode,
        Invalid_Mode,
    } CameraMode;

public:
    // 相机内参
    static double cx;
    static double cy;
    static double fx;
    static double fy;

#define TQC_DEFAULT_CAM_WIDTH  640
#define TQC_DEFAULT_CAM_HEIGHT 480

public:
    CCamera(int nLeftCamIndex, int nRightCamIndex, CameraMode mode);
    CCamera(int nCamIndex);
    ~CCamera();

    bool    IsValid();
    void    InitCameraParam(int width, int height);
    void    OpenCamera();
    bool    IsCameraOpened();
    void    CloseCamera();
    void    ReadCamera(Mat &frame);
    void    ReadCamera(Mat &leftFrame, Mat &rightFrame);

public:
    int m_nCamWidth;
    int m_nCamHeight;

private:
    bool         m_bValid;
    CameraMode   m_camMode;
    int          m_nLeftCamIndex;
    int          m_nRightCamIndex;
    VideoCapture m_leftCamera;
    VideoCapture m_rightCamera;
};
#endif // TQCCAMERA_H