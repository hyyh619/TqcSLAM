#include "TqcCamera.h"

using namespace cv;

double CCamera::cx = 325.5;
double CCamera::cy = 253.5;
double CCamera::fx = 518.0;
double CCamera::fy = 519.0;

int GetCameraDeviceCount()
{
    int nCount = 0;
    int nMax   = 10;

    for (int i = 0; i<nMax; i++)
    {
        VideoCapture tmpCam(i);
        bool         res = tmpCam.isOpened();
        if (res == true)
        {
            nCount++;
        }
    }

    return nCount;
}

CCamera::CCamera(int nLeftCamIndex, int nRightCamIndex, CameraMode mode)
{
    InitCameraParam(TQC_DEFAULT_CAM_WIDTH, TQC_DEFAULT_CAM_HEIGHT);

    m_bValid  = true;
    m_camMode = mode;

    switch (mode)
    {
    case Single_Camera_Mode:
        m_nLeftCamIndex  = nLeftCamIndex;
        m_nRightCamIndex = -1;
        break;

    case Stereo_Camera_Mode:
        m_nLeftCamIndex  = nLeftCamIndex;
        m_nRightCamIndex = nRightCamIndex;
        break;

    case RGBD_Camera_Mode:
        m_nLeftCamIndex  = nLeftCamIndex;
        m_nRightCamIndex = nRightCamIndex;
        break;

    default:
        m_bValid = false;
        break;
    }
}

CCamera::CCamera(int nCamIndex)
{
    InitCameraParam(TQC_DEFAULT_CAM_WIDTH, TQC_DEFAULT_CAM_HEIGHT);

    m_bValid         = true;
    m_nLeftCamIndex  = nCamIndex;
    m_nRightCamIndex = -1;
    m_camMode        = Single_Camera_Mode;
}

CCamera::~CCamera()
{}

void CCamera::InitCameraParam(int width, int height)
{
    m_nCamWidth  = width;
    m_nCamHeight = height;
}

bool CCamera::IsValid()
{
    return m_bValid;
}

void CCamera::OpenCamera()
{
    switch (m_camMode)
    {
    case Single_Camera_Mode:
        m_leftCamera.open(m_nLeftCamIndex);
        m_leftCamera.set(CV_CAP_PROP_FRAME_WIDTH, m_nCamWidth);
        m_leftCamera.set(CV_CAP_PROP_FRAME_HEIGHT, m_nCamHeight);
        break;

    case Stereo_Camera_Mode:
        m_leftCamera.open(m_nLeftCamIndex);
        m_leftCamera.set(CV_CAP_PROP_FRAME_WIDTH, m_nCamWidth);
        m_leftCamera.set(CV_CAP_PROP_FRAME_HEIGHT, m_nCamHeight);

        m_rightCamera.open(m_nRightCamIndex);
        m_rightCamera.set(CV_CAP_PROP_FRAME_WIDTH, m_nCamWidth);
        m_rightCamera.set(CV_CAP_PROP_FRAME_HEIGHT, m_nCamHeight);
        break;

    case RGBD_Camera_Mode:
        break;

    default:
        m_bValid = false;
        break;
    }
}

void CCamera::CloseCamera()
{}

void CCamera::ReadCamera(Mat &frame)
{
    m_leftCamera >> frame;
}

void CCamera::ReadCamera(Mat &leftFrame, Mat &rightFrame)
{
    m_leftCamera >> leftFrame;
    m_rightCamera >> rightFrame;
}

bool CCamera::IsCameraOpened()
{
    switch (m_camMode)
    {
    case Single_Camera_Mode:
        return m_leftCamera.isOpened();

    case Stereo_Camera_Mode:
        return m_leftCamera.isOpened() && m_rightCamera.isOpened();

    case RGBD_Camera_Mode:
        return false;

    default:
        return false;
    }
}