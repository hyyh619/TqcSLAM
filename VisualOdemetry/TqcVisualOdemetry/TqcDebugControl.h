#ifndef TQCDEBUGCONTROL_H
#define TQCDEBUGCONTROL_H

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/optimizable_graph.h>

class CDebugControl
{
public:
    CDebugControl();
    ~CDebugControl();

    void OutputPoseR(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d::LinearMatrixType r);
    void OutputPoseT(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d::TranslationPart t);
    void OutputMatrix(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d pose);
    void OutputGlobalT(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d pose);
    void OutputGlobalR(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d pose);

    void OutputPoseT(char *buf, int nBufSize, int nIndex, double *t);
    void OutputPoseR(char *buf, int nBufSize, int nIndex, double *r);
    void OutputMatrix(char *buf, int nBufSize, int nIndex, double *pose);
    void OutputGlobalT(char *buf, int nBufSize, int nIndex, double *pose);
    void OutputGlobalR(char *buf, int nBufSize, int nIndex, double *pose);

private:
    bool CheckOutputBuf(char *buf, int nBufSize);

public:
    bool m_bMatrixToggle;
    bool m_bPoseTToggle;
    bool m_bPoseRToggle;
    bool m_bGlobalTToggle;
    bool m_bGlobalRToggle;
};
#endif // TQCDEBUGCONTROL_H