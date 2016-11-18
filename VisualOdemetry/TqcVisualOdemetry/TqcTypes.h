#ifndef __TQC_TYPES_H
#define __TQC_TYPES_H

#include <vector>
#include <queue>
#include <QString>
#include <QQueue>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// for g2o
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace cv;

class CPose
{
public:
    double globalMatrix[16];
    double m[16];
    double t[3];
    double r[4][4];
    int    nIndex;

public:
    CPose();
    ~CPose();

    void SetIndex(int i);
    void SetGlobalMatrix(double *matrix);
    void SetMatrix(double *matrix);
    void SetTranslation(double x, double y, double z);
    void SetRotation(double *rotation);
    void Save(QDataStream &out);
    void Load(QDataStream &in);
};

typedef vector<CPose*>            PoseArray;
typedef vector<QString*>          StringArray;
typedef QQueue<Mat>               MatQueue;
typedef QQueue<Eigen::Isometry3d> PoseQueue;
#endif // __TQC_TYPES_H