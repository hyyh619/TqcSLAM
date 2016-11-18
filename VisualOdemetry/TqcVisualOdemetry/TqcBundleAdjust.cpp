#include <QDebug>
#include "TqcDefines.h"
#include "TqcBundleAdjust.h"
#include "TqcCamera.h"

using namespace std;
using namespace cv;

bool PoseEstimate(Mat &prev, Mat &cur, Eigen::Isometry3d &pose, bool bDraw, Mat &match, Mat &goodMatch)
{
    // 读取图像
    Mat img1 = prev;
    Mat img2 = cur;

    pose.setIdentity();

    // 找到对应点
    vector<cv::Point2f> pts1, pts2;
    if (FindCorrespondingPoints(img1, img2, pts1, pts2, bDraw, match, goodMatch) == false)
    {
        // qDebug() << "匹配点不够！" << endl;
        return false;
    }

    cout << "找到了" << pts1.size() << "组对应特征点。" << endl;
    // 构造g2o中的图
    // 先构造求解器
    g2o::SparseOptimizer optimizer;
    // 使用Cholmod中的线性方程求解器
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver = new  g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    // 6*3 的参数
    g2o::BlockSolver_6_3 *block_solver = new g2o::BlockSolver_6_3(linearSolver);
    // L-M 下降
    g2o::OptimizationAlgorithmLevenberg *algorithm = new g2o::OptimizationAlgorithmLevenberg(block_solver);

    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(false);

    // 添加节点
    // 两个位姿节点
    for (int i = 0; i<2; i++)
    {
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if (i == 0)
            v->setFixed(true);   // 第一个点固定为零

        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate(g2o::SE3Quat());
        optimizer.addVertex(v);
    }

    // 很多个特征点的节点
    // 以第一帧为准
    for (size_t i = 0; i<pts1.size(); i++)
    {
        g2o::VertexSBAPointXYZ *v = new g2o::VertexSBAPointXYZ();
        v->setId(2 + i);
        // 由于深度不知道，只能把深度设置为1了
        double z = 1;
        double x = (pts1[i].x - CCamera::cx) * z / CCamera::fx;
        double y = (pts1[i].y - CCamera::cy) * z / CCamera::fy;
        v->setMarginalized(true);
        v->setEstimate(Eigen::Vector3d(x, y, z));
        optimizer.addVertex(v);
    }

    // 准备相机参数
    g2o::CameraParameters *camera = new g2o::CameraParameters(CCamera::fx, Eigen::Vector2d(CCamera::cx, CCamera::cy), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

    // 准备边
    // 第一帧
    vector<g2o::EdgeProjectXYZ2UV*> edges;

    for (size_t i = 0; i<pts1.size(); i++)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i + 2)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0)));
        edge->setMeasurement(Eigen::Vector2d(pts1[i].x, pts1[i].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    // 第二帧
    for (size_t i = 0; i<pts2.size(); i++)
    {
        g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(i + 2)));
        edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(1)));
        edge->setMeasurement(Eigen::Vector2d(pts2[i].x, pts2[i].y));
        edge->setInformation(Eigen::Matrix2d::Identity());
        edge->setParameterId(0, 0);
        // 核函数
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
        edges.push_back(edge);
    }

    // cout << "开始优化" << endl;
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    // cout << "优化完毕" << endl;

    // 我们比较关心两帧之间的变换矩阵
    g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(1));

    pose = v->estimate();
    cout << "Pose=" << endl << pose.matrix() << endl;

    return true;
}

bool FindCorrespondingPoints(const cv::Mat &img1, const cv::Mat &img2, vector<cv::Point2f> &points1, vector<cv::Point2f> &points2, bool bDraw, Mat &match, Mat &goodMatch)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    vector<cv::KeyPoint> kp1, kp2;
    cv::Mat              desp1, desp2;

    if (img1.cols != img2.cols ||
        img1.type() != img2.type())
    {
        qDebug() << "img cols " << img1.cols << " != " << img2.cols;
        qDebug() << "img.type " << img1.type() << " != " << img2.type() << endl;
        return false;
    }

    orb->detect(img1, kp1);
    orb->compute(img1, kp1, desp1);
    orb->detect(img2, kp2);
    orb->compute(img2, kp2, desp2);

    if (kp1.size() <= 20 || kp2.size() <= 20)
    {
        qDebug() << "分别找到了" << kp1.size() << "和" << kp2.size() << "个特征点" << endl;
        return false;
    }

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    double                      knn_match_ratio = 0.8;
    vector<vector<cv::DMatch> > matches_knn;
    matcher->knnMatch(desp1, desp2, matches_knn, 2);
    vector<cv::DMatch> matches;

    for (size_t i = 0; i<matches_knn.size(); i++)
    {
        if (matches_knn[i][0].distance < knn_match_ratio * matches_knn[i][1].distance)
            matches.push_back(matches_knn[i][0]);
    }

    // 筛选匹配，把距离太大的去掉
    // 这里使用的准则是去掉大于四倍最小距离的匹配
    vector<cv::DMatch> goodMatches;
    float              minDis = 9999;

    for (size_t i = 0; i<matches.size(); i++)
    {
        if (matches[i].distance == 0.0f)
            continue;
        
        if (matches[i].distance < minDis)
            minDis = matches[i].distance;
    }

    for (size_t i = 0; i<matches.size(); i++)
    {
        if (matches[i].distance < TQC_BA_DISTANCE_SCALE * minDis)
            goodMatches.push_back(matches[i]);
    }

    if (matches.size() <= 20) // 匹配点太少
    {
        qDebug() << "匹配点太少：" << matches.size() << endl;
        return false;
    }

#if 0
    for (auto m:matches)
    {
        points1.push_back(kp1[m.queryIdx].pt);
        points2.push_back(kp2[m.trainIdx].pt);
    }
#endif
    
    // We uses good matches.
    for (auto m:goodMatches)
    {
        points1.push_back(kp1[m.queryIdx].pt);
        points2.push_back(kp2[m.trainIdx].pt);
    }

    if (bDraw)
    {
        drawMatches(img1, kp1, img2, kp2, matches, match);
        drawMatches(img1, kp1, img2, kp2, goodMatches, goodMatch);
    }

    return true;
}

CBundleAdjustThread::CBundleAdjustThread()
{
    m_nMatQueueSleep = 5;    // sleep 5ms
    m_bPause         = false;
    m_bDrawMatch     = false;

    m_prevFrame = Mat();
}

void CBundleAdjustThread::Pause(bool bPause)
{
    m_bPause = bPause;
}

void CBundleAdjustThread::SetDrawMatch(bool bDraw)
{
    m_bDrawMatch = bDraw;
}

void CBundleAdjustThread::run()
{
    Mat               frame;
    Mat               drawMatch;
    Mat               drawGoodMatch;
    Eigen::Isometry3d pose;

    m_prevFrame = Mat();
    ReleaseMatQueue();
    ReleasePoseQueue();
    m_bStop  = false;
    m_bPause = false;

    while (!m_bStop)
    {
        if (m_bPause)
        {
            msleep(20);
            continue;
        }

        if (!PopMat(frame))
        {
            msleep(m_nMatQueueSleep);
            if (m_nMatQueueSleep < TQC_BA_MAT_QUEUE_SLEEP_TIME_MAX)
                m_nMatQueueSleep += 5;
        }
        else
        {
            if (m_nMatQueueSleep > TQC_BA_MAT_QUEUE_SLEEP_TIME_MIN)
            {
                m_nMatQueueSleep -= 5;
            }

            if (!m_prevFrame.empty())
            {
                if (PoseEstimate(m_prevFrame, frame, pose, m_bDrawMatch, drawMatch, drawGoodMatch))
                {
                    PushPose(pose);

                    // Update match mat
                    if (m_bDrawMatch)
                    {
                        m_drawMatchMatMutex.lock();
                        m_drawMatchMat = drawMatch;
                        m_drawGoodMatchMat = drawGoodMatch;
                        m_drawMatchMatMutex.unlock();
                    }
                }
            }

            m_prevFrame = frame;
        }
    }

    m_bStop = false;
}

void CBundleAdjustThread::GetDrawMatchMat(Mat &match, Mat &goodMatch)
{
    m_drawMatchMatMutex.lock();
    match = m_drawMatchMat;
    goodMatch = m_drawGoodMatchMat;
    m_drawMatchMatMutex.unlock();
}

int CBundleAdjustThread::GetMatQueueSize()
{
    int nSize = 0;

    m_matQueueMutex.lock();
    nSize = m_matQueue.size();
    m_matQueueMutex.unlock();

    return nSize;
}

bool CBundleAdjustThread::PushMat(Mat &frame)
{
    bool res = false;

    m_matQueueMutex.lock();
    if (m_matQueue.size() < TQC_BA_MAT_QUEUE_SIZE)
    {
        m_matQueue.enqueue(frame);
        res = true;
    }

    m_matQueueMutex.unlock();

    return res;
}

bool CBundleAdjustThread::PopMat(Mat &frame)
{
    bool res = false;

    m_matQueueMutex.lock();
    if (!m_matQueue.isEmpty())
    {
        frame = m_matQueue.dequeue();
        res   = true;
    }

    m_matQueueMutex.unlock();

    return res;
}

bool CBundleAdjustThread::PushPose(Eigen::Isometry3d &pose)
{
    bool res = true;

    m_poseQueueMutex.lock();
    m_poseQueue.enqueue(pose);
    m_poseQueueMutex.unlock();

    return res;
}

bool CBundleAdjustThread::PopPose(Eigen::Isometry3d &pose)
{
    bool res = false;

    m_poseQueueMutex.lock();
    if (!m_poseQueue.isEmpty())
    {
        pose = m_poseQueue.dequeue();
        res  = true;
    }

    m_poseQueueMutex.unlock();

    return res;
}

void CBundleAdjustThread::ReleaseMatQueue()
{
    m_matQueue.clear();
}

void CBundleAdjustThread::ReleasePoseQueue()
{
    m_poseQueue.clear();
}
