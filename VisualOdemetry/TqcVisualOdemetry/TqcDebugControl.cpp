/*************************************************************************************
* Control the debug output. It includes the following items
*  1. Output original matrix between previous frame and current frame.
*  2. Output pose information, including translation and rotation.
*  3. Output global pose information which is according to world coordinate system.
*************************************************************************************/

#include "TqcDefines.h"
#include "TqcDebugControl.h"

CDebugControl::CDebugControl()
{
    m_bMatrixToggle  = false;
    m_bPoseTToggle   = false;
    m_bPoseRToggle   = false;
    m_bGlobalTToggle = true;
    m_bGlobalRToggle = false;
}

CDebugControl::~CDebugControl()
{}

bool CDebugControl::CheckOutputBuf(char *buf, int nBufSize)
{
    if (buf == NULL || nBufSize == 0)
        return false;

    memset(buf, 0, nBufSize);
    return true;
}

void CDebugControl::OutputPoseR(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d::LinearMatrixType r)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_ROTATION,
            nIndex,
            r(0, 0), r(0, 1), r(0, 2),
            r(1, 0), r(1, 1), r(1, 2),
            r(2, 0), r(2, 1), r(2, 2));
}

void CDebugControl::OutputPoseT(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d::TranslationPart t)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_TRANSLATION,
            nIndex, t(0), t(1), t(2));
}

void CDebugControl::OutputMatrix(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d pose)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n",
            TQC_DEBUG_STR_MATRIX,
            nIndex,
            pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
            pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
            pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3),
            pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3));
}

void CDebugControl::OutputGlobalT(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d pose)
{
    Eigen::Isometry3d::TranslationPart t = pose.translation();

    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_GLOBAL_T,
            nIndex, t(0), t(1), t(2));
}

void CDebugControl::OutputGlobalR(char *buf, int nBufSize, int nIndex, Eigen::Isometry3d pose)
{
    Eigen::Isometry3d::LinearMatrixType r = pose.rotation();

    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_GLOBAL_R,
            nIndex,
            r(0, 0), r(0, 1), r(0, 2),
            r(1, 0), r(1, 1), r(1, 2),
            r(2, 0), r(2, 1), r(2, 2));
}

void CDebugControl::OutputPoseR(char *buf, int nBufSize, int nIndex, double *r)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_ROTATION,
            nIndex,
            r[0 * 4 + 0], r[0 * 4 + 1], r[0 * 4 + 2],
            r[1 * 4 + 0], r[1 * 4 + 1], r[1 * 4 + 2],
            r[2 * 4 + 0], r[2 * 4 + 1], r[2 * 4 + 2]);
}

void CDebugControl::OutputPoseT(char *buf, int nBufSize, int nIndex,  double *t)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_TRANSLATION,
            nIndex, t[0], t[1], t[2]);
}

void CDebugControl::OutputMatrix(char *buf, int nBufSize, int nIndex, double *pose)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n"
            "    %+02.8f %+02.8f %+02.8f %+02.8f\n",
            TQC_DEBUG_STR_MATRIX,
            nIndex,
            pose[0 * 4 + 0], pose[0 * 4 + 1], pose[0 * 4 + 2], pose[0 * 4 + 3],
            pose[1 * 4 + 0], pose[1 * 4 + 1], pose[1 * 4 + 2], pose[1 * 4 + 3],
            pose[2 * 4 + 0], pose[2 * 4 + 1], pose[2 * 4 + 2], pose[2 * 4 + 3],
            pose[3 * 4 + 0], pose[3 * 4 + 1], pose[3 * 4 + 2], pose[3 * 4 + 3]);
}

void CDebugControl::OutputGlobalT(char *buf, int nBufSize, int nIndex, double *pose)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_GLOBAL_T,
            nIndex, pose[0 * 4 + 3], pose[1 * 4 + 3], pose[2 * 4 + 3]);
}

void CDebugControl::OutputGlobalR(char *buf, int nBufSize, int nIndex, double *pose)
{
    if (!CheckOutputBuf(buf, nBufSize))
        return;

    sprintf(buf, "%s%04d:\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n"
            "    %+02.8f, %+02.8f, %+02.8f\n",
            TQC_DEBUG_STR_GLOBAL_R,
            nIndex,
            pose[0 * 4 + 0], pose[0 * 4 + 1], pose[0 * 4 + 2],
            pose[1 * 4 + 0], pose[1 * 4 + 1], pose[1 * 4 + 2],
            pose[2 * 4 + 0], pose[2 * 4 + 1], pose[2 * 4 + 2]);
}