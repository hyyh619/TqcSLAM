#ifndef BUNDLEADJUST_H
#define BUNDLEADJUST_H

// for std
#include <iostream>

// for opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <g2o/core/sparse_optimizer.h>

// for QT
#include <QMutex>

#include "TqcTypes.h"
#include <opencv2/core/eigen.hpp>
#include "TqcThread.h"

using namespace cv;
using namespace std;

// 寻找两个图像中的对应点，像素坐标系
// 输入：img1, img2 两张图像
// 输出：points1, points2, 两组对应的2D点
bool FindCorrespondingPoints(const Mat &img1, const Mat &img2, vector<Point2f> &points1, vector<Point2f> &points2, bool bDraw, Mat &match, Mat &goodMatch);
bool PoseEstimate(Mat &prev, Mat &cur, Eigen::Isometry3d &pose, bool bDraw, Mat &match, Mat &goodMatch);
double NormOfTransform(cv::Mat rvec, cv::Mat tvec);

class CBundleAdjustThread : public CThread
{
public:
    CBundleAdjustThread();
    void run();
    bool PushMat(Mat &frame);
    bool PopMat(Mat &frame);
    int  GetMatQueueSize();
    void ReleaseMatQueue();
    bool PushPose(Eigen::Isometry3d &pose);
    bool PopPose(Eigen::Isometry3d &pose);
    void ReleasePoseQueue();
    void Pause(bool bPause);
    void SetDrawMatch(bool bDraw);
    void GetDrawMatchMat(Mat &match, Mat &goodMatch);

private:
    QMutex    m_matQueueMutex;
    MatQueue  m_matQueue;
    int       m_nMatQueueSleep;
    Mat       m_prevFrame;
    QMutex    m_poseQueueMutex;
    PoseQueue m_poseQueue;
    bool      m_bPause;
    bool      m_bDrawMatch;
    QMutex    m_drawMatchMatMutex;
    Mat       m_drawMatchMat;
    Mat       m_drawGoodMatchMat;
};
#endif // BUNDLEADJUST_H
