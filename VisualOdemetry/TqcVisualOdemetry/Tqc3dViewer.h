#ifndef TQC3DVIEWER_H
#define TQC3DVIEWER_H

#include <vector>
#include <string>
#include <QGLViewer/qglviewer.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/hyper_graph_action.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <QObject>
#include <QTextBrowser>
#include "TqcDefines.h"
#include "TqcTypes.h"
#include "TqcDebugControl.h"
#include "OutputDialog.h"

using namespace g2o;
using namespace std;


class Viewer : public QGLViewer
{
public:
    Viewer(QWidget *parent);
    ~Viewer();
    void  SetUpdateDisplay(bool updateDisplay);
    void  Release();
    void  Save(string fileName);
    void  Load(string fileName);
    void  SetDebugDialog(OutputDialog *pDialog);
    void  SetDebugOutput(QTextBrowser *debugOutput);

protected:
    virtual void init();
    virtual void draw();
    virtual void fastDraw();
    virtual QString helpString() const;
    virtual void drawWithNames();
    virtual void postSelection(const QPoint &point);

private:
    void DrawGraph(bool bSelect);
    void ApplyGraphDraw(bool bSelect);
    void DrawArrow2D(double x, double y, double z, float head_width, float head_len);
    bool IsNeedDraw();
    void DrawPoses(PoseArray *pArr, int drawColor, bool bSelect);

public:
    SparseOptimizer *m_graph;
    PoseArray       m_poses;
    PoseArray       m_loadPoseArray[TQC_LOAD_POSES_ARRAY_NUM];
    int             m_nLoaded;
    static float    m_drawColor[TQC_POSE_COLOR_NUM][3];
    int             m_nObjName;

protected:
    GLuint         m_drawList;
    bool           m_bUpdateDisplay;
    qglviewer::Vec m_lastCamPos;

private:
    OutputDialog  *m_pDebugDialog;
    QTextBrowser  *m_pDebugOutput;
    float         m_fScale;
    CDebugControl m_debugCtrl;
};
#endif // TQC3DVIEWER_H