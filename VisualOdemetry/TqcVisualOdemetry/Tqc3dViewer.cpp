#include <math.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/optimizable_graph.h>
#include <QDebug.h>
#include <QFile.h>
#include <QString.h>

#include "Tqc3dViewer.h"

using namespace std;
using namespace qglviewer;

namespace
{
/**
 * \brief helper for setting up a camera for qglviewer
 */
class StandardCamera : public qglviewer::Camera
{
public:
    StandardCamera() : _standard(true) {};

    qreal zNear() const
    {
        if (_standard)
            return qreal(0.001);
        else
            return Camera::zNear();
    }

    qreal zFar() const
    {
        if (_standard)
            return qreal(10000.0);
        else
            return Camera::zFar();
    }

    bool standard() const
    {
        return _standard;
    }
    void setStandard(bool s)
    {
        _standard = s;
    }

private:
    bool _standard;
};
}   // end anonymous namespace

float Viewer::m_drawColor[TQC_POSE_COLOR_NUM][3] =
{
    {TQC_POSE_VERTEX_COLOR},
    {TQC_POSE_LATEST_VERTEX_COLOR},
    {TQC_LOADED_POSE_COLOR1},
    {TQC_LOADED_POSE_COLOR2},
    {TQC_LOADED_POSE_COLOR3},
    {TQC_LOADED_POSE_COLOR4},
    {TQC_LOADED_POSE_COLOR5}
};

Viewer::Viewer(QWidget *parent)
    : QGLViewer(parent)
{
    // restoreStateFromFile();
    // help();

    m_fScale = TQC_POSE_SCALE;
    m_graph  = NULL;
    m_poses.clear();

    for (int i = 0; i<TQC_LOAD_POSES_ARRAY_NUM; i++)
    {
        m_loadPoseArray[i].clear();
    }

    m_nLoaded = 0;
}

Viewer::~Viewer()
{
    glDeleteLists(m_drawList, 1);

    for (vector<CPose*>::iterator it = m_poses.begin(); it != m_poses.end();)
    {
        CPose *p = *it;
        it = m_poses.erase(it);
        delete p;
    }

    for (int i = 0; i<TQC_LOAD_POSES_ARRAY_NUM; i++)
    {
        PoseArray *pArr = &m_loadPoseArray[i];

        for (vector<CPose*>::iterator it = pArr->begin(); it != pArr->end();)
        {
            CPose *p = *it;
            it = pArr->erase(it);
            delete p;
        }
    }
}

void Viewer::init()
{
    // Increase the material shininess, so that the difference between
    // the two versions of the spiral is more visible.
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
    GLfloat specular_color[4] = { 0.8f, 0.8f, 0.8f, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

    restoreStateFromFile();

    // gray color: setBackgroundColor(QColor(182, 190, 207));
    setBackgroundColor(QColor::fromRgb(51, 51, 51));

    // some default settings i like
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_NORMALIZE);
    // glEnable(GL_CULL_FACE);
    glShadeModel(GL_FLAT);
    // glShadeModel(GL_SMOOTH);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    setGridIsDrawn(true);
    setAxisIsDrawn(false);

    // don't save state
    setStateFileName(QString::null);

    // mouse bindings
#ifdef QGLVIEWER_DEPRECATED_MOUSEBINDING
    setMouseBinding(Qt::NoModifier, Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::NoModifier, Qt::MidButton, CAMERA, TRANSLATE);
#else
    setMouseBinding(Qt::RightButton, CAMERA, ZOOM);
    setMouseBinding(Qt::MidButton, CAMERA, TRANSLATE);
#endif

    // replace camera
    qglviewer::Camera *oldcam = camera();
    qglviewer::Camera *cam    = new StandardCamera();
    setCamera(cam);
    cam->setPosition(qglviewer::Vec(0., 0., 1.));
    cam->setUpVector(qglviewer::Vec(0., 1., 0.));
    cam->lookAt(qglviewer::Vec(0., 0., 0.));
    delete oldcam;

    // getting a display list
    m_drawList = glGenLists(1);
}

void Viewer::DrawPoses(PoseArray *pArr, int drawColor, bool bSelect = false)
{
    if (pArr->empty())
        return;

    vector<CPose*>::iterator it    = pArr->begin();
    CPose                    *prev = *it++;
    CPose                    *p    = NULL;
    int                      nNum  = pArr->size();
    Camera                   *pCam = camera();
    float                    fArrowWidth;
    float                    fArrowLen;
    float                    fCamDist;
    float                    fX, fY, fZ;
    qglviewer::Vec           position = pCam->position();
    char                     buf[32];

    memset(buf, 0, 32);

    for (int i = 2; it != pArr->end(); ++it, ++i)
    {
        p = *it;

        // Calc camera distance from current vertex.
        // fCamDist = position.norm();
        fX       = position.x - p->globalMatrix[4 * 0 + 3];
        fY       = position.y - p->globalMatrix[4 * 1 + 3];
        fZ       = position.z - p->globalMatrix[4 * 2 + 3];
        fCamDist = sqrt(fX * fX + fY * fY + fZ * fZ);
        if (fCamDist > 1.0f)
            fCamDist = 1.0f;

        fArrowLen   = TQC_POSE_ARROW_LEN * fCamDist;
        fArrowWidth = TQC_POSE_ARROW_WIDTH * fCamDist;

        glPushMatrix();
        glMultTransposeMatrixd(p->globalMatrix);

        // qglviewer::Vec screenPos = camera()->projectedCoordinatesOf(qglviewer::Vec(prev->t[0], prev->t[1], prev->t[2]));
        // drawText((int)screenPos[0], (int)screenPos[1], "My Object");
        glColor3f(TQC_POSE_TEXT_COLOR);
        sprintf(buf, "v%d", p->nIndex);
        renderText(prev->t[0], prev->t[1], prev->t[2], buf);

        glColor3f(m_drawColor[drawColor][0],
                  m_drawColor[drawColor][1],
                  m_drawColor[drawColor][2]);

        // For the latest arrow, we use red.
        if (i == nNum)
        {
            glColor3f(TQC_POSE_LATEST_VERTEX_COLOR);
        }

        if (bSelect)
        {
            glPushName(m_nObjName);
        }

        if (selectedName() == m_nObjName)
        {
            glColor3f(TQC_POSE_SELECTED_COLOR);
        }

        m_nObjName++;

        DrawArrow2D(prev->t[0], prev->t[1], prev->t[2], fArrowWidth, fArrowLen);

        if (bSelect)
        {
            glPopName();
        }

        glPopMatrix();
    }
}

void Viewer::ApplyGraphDraw(bool bSelect = false)
{
    // double total[3] = {0, 0, 0};

    glDisable(GL_BLEND);
    glDisable(GL_LINE_SMOOTH);
    glDisable(GL_DEPTH);
    glDisable(GL_LIGHTING);

    glColor3f(TQC_POSE_VERTEX_COLOR);
    glLineWidth(TQC_POSE_EDGE_WIDTH);

#if 0
    // Draw lines
    glBegin(GL_LINE_STRIP);

    for (vector<CPose*>::iterator it = m_poses.begin(); it != m_poses.end(); ++it)
    {
        CPose *p = *it;

        total[0] += p->t[0] * m_fScale;
        total[1] += p->t[1] * m_fScale;
        total[2] += p->t[2];

        glMultTransposeMatrixd((const double*)p->r);
        glVertex3f(total[0], total[1], total[2]);
    }

    glEnd();

    // Draw arrows
    total[0] = 0;
    total[1] = 0;
    total[2] = 0;

    // Move arrow to correct position
    glTranslated(-TQC_POSE_ARROW_LEN, 0.0, 0.0);

    for (vector<CPose*>::iterator it = m_poses.begin(); it != m_poses.end(); ++it)
    {
        CPose *p = *it;

        double x = p->t[0] * m_fScale;
        double y = p->t[1] * m_fScale;
        double z = p->t[2];

        // glMultTransposeMatrixd((const double*)p->r);
        glTranslated(x, y, z);
        // glMultMatrixd((const double*)p->r);
        // glMultTransposeMatrixd((const double*)p->r);
        // glMultMatrixd(p->m);

        DrawArrow2D(0.2, TQC_POSE_ARROW_WIDTH, TQC_POSE_ARROW_LEN);
    }
#endif

#if 0
    vector<CPose*>::iterator it    = m_poses.begin();
    CPose                    *prev = *it++;
    CPose                    *p    = NULL;

    total[0] += prev->t[0] * m_fScale;
    total[1] += prev->t[1] * m_fScale;
    total[2] += prev->t[2];

    for (; it != m_poses.end(); ++it)
    {
        p = *it;

        glBegin(GL_LINE_STRIP);
        glVertex3f(total[0], total[1], total[2]);
        total[0] += p->t[0] * m_fScale;
        total[1] += p->t[1] * m_fScale;
        total[2] += p->t[2];
        glMultTransposeMatrixd((const double*)p->r);
        glVertex3f(total[0], total[1], total[2]);
        glEnd();

        DrawArrow2D(total[0] - TQC_POSE_ARROW_LEN, total[1], total[2], TQC_POSE_ARROW_WIDTH, TQC_POSE_ARROW_LEN);
    }
#endif

    m_nObjName = 0;

    DrawPoses(&m_poses, TQC_POSE_COLOR_VIDEO_INDEX, bSelect);

    // Draw Loaded graph file
    for (int i = 0; i<m_nLoaded; i++)
    {
        DrawPoses(&m_loadPoseArray[i], TQC_POSE_COLOR_LOAD_INDEX + i, bSelect);
    }
}

void Viewer::DrawGraph(bool bSelect = false)
{
    if (!IsNeedDraw())
        return;

    Camera         *pCam    = camera();
    qglviewer::Vec position = pCam->position();

    // Camera position moves, we should redraw the arrow with new size.
    /*if ((m_lastCamPos != position) ||
        m_bUpdateDisplay)
       {
        m_bUpdateDisplay = false;
        glNewList(m_drawList, GL_COMPILE_AND_EXECUTE);
        ApplyGraphDraw();
        glEndList();
       }
       else
       {
        glCallList(m_drawList);
       }*/

    m_bUpdateDisplay = false;
    ApplyGraphDraw(bSelect);

    m_lastCamPos = position;
}

void Viewer::draw()
{
    DrawGraph();
}

void Viewer::drawWithNames()
{
    DrawGraph(true);
}

void Viewer::fastDraw()
{
    DrawGraph();
}

bool Viewer::IsNeedDraw()
{
    if (!m_poses.empty())
        return true;

    for (int i = 0; i<m_nLoaded; i++)
    {
        if (!m_loadPoseArray[i].empty())
            return true;
    }

    return false;
}

void Viewer::SetUpdateDisplay(bool updateDisplay)
{
    m_bUpdateDisplay = updateDisplay;
}

QString Viewer::helpString() const
{
    QString text("<h2>F a s t D r a w</h2>");

    text += "The <code>fastDraw()</code> function is called instead of <code>draw()</code> when the camera ";
    text += "is manipulated. Providing such a simplified version of <code>draw()</code> allows for interactive ";
    text += "frame rates when the camera is moved, even for very complex scenes.";
    return text;
}

void Viewer::DrawArrow2D(double x, double y, double z, float head_width, float head_len)
{
    glNormal3f(0.f, 0.f, 1.f);
    glBegin(GL_TRIANGLES);
    glVertex3d(x, y, z);
    glVertex3d(x + head_len, y + 0.5f * head_width, z);
    glVertex3d(x + head_len, y - 0.5f * head_width, z);
    glEnd();
}

void Viewer::Release()
{
    if (m_graph)
    {
        // Release poses information.
        for (vector<CPose*>::iterator it = m_poses.begin(); it != m_poses.end();)
        {
            CPose *p = *it;
            it = m_poses.erase(it);
            delete p;
        }

        // Release grapher
        m_graph->clear();

        delete m_graph;
    }

    SetUpdateDisplay(true);
}

void Viewer::Save(string fileName)
{
    QString qStr = QString(fileName.c_str());
    QFile   file(qStr);

    if (file.open(QIODevice::WriteOnly)) // 以文本文式写入
    {
        QDataStream out(&file);

        // Save number of poses.
        out << (int)m_poses.size();

        for (vector<CPose*>::iterator it = m_poses.begin(); it != m_poses.end(); it++)
        {
            CPose *p = *it;
            p->Save(out);
        }

        file.close();
    }
    else
    {
        qDebug() << "Cannot save file: " << fileName.c_str();
    }
}

void Viewer::Load(string fileName)
{
    QString qStr = QString(fileName.c_str());
    QFile   file(qStr);
    int     nNum;

    if (file.open(QIODevice::ReadOnly)) // 以文本文式写入
    {
        QDataStream in(&file);

        if (m_nLoaded > TQC_LOAD_POSES_ARRAY_NUM)
        {
            file.close();
            return;
        }

        m_nLoaded++;

        in >> nNum;

        m_pDebugDialog->UpdateChannelName(m_nLoaded, qStr);

        for (int i = 0; i<nNum; i++)
        {
            CPose *p = new CPose;
            p->Load(in);
            m_loadPoseArray[m_nLoaded - 1].push_back(p);

            m_pDebugDialog->LoadInformFromPose(m_nLoaded, p);
        }

        SetUpdateDisplay(true);
        updateGL();

        file.close();
    }
    else
    {
        qDebug() << "Cannot save file: " << fileName.c_str();
    }
}

void Viewer::SetDebugDialog(OutputDialog *pDialog)
{
    m_pDebugDialog = pDialog;
}

void Viewer::SetDebugOutput(QTextBrowser *debugOutput)
{
    m_pDebugOutput = debugOutput;
}

void Viewer::postSelection(const QPoint &point)
{
    Viewer    *pView       = this;
    CPose     *p           = NULL;
    PoseArray *pArr        = NULL;
    int       nSelect      = selectedName();
    int       nVertexIndex = 0;
    int       nSize;
    char      buf[256];

    if (nSelect == -1)
        return;

    do
    {
        nSize = (int)pView->m_poses.size();
        if (nSelect < nSize)
        {
            p            = pView->m_poses[nSelect];
            nVertexIndex = nSelect + 1;
            break;
        }

        nSelect -= nSize;

        for (int i = 0; i<TQC_LOAD_POSES_ARRAY_NUM; i++)
        {
            pArr  = &pView->m_loadPoseArray[i];
            nSize = (int)pArr->size();
            if (nSelect < nSize)
            {
                p            = (*pArr)[nSelect];
                nVertexIndex = nSelect + 1;
                break;
            }

            nSelect -= nSize;
        }
    }
    while (false);

    if (!p)
        return;

    // Output debug info to DebugOutput view.
    m_debugCtrl.OutputPoseT(buf, 256, nVertexIndex, p->t);
    m_pDebugOutput->append(tr(buf));
    m_debugCtrl.OutputPoseR(buf, 256, nVertexIndex, (double*)p->r[0]);
    m_pDebugOutput->append(tr(buf));

    m_debugCtrl.OutputGlobalT(buf, 256, nVertexIndex, p->globalMatrix);
    m_pDebugOutput->append(tr(buf));
    m_debugCtrl.OutputGlobalR(buf, 256, nVertexIndex, p->globalMatrix);
    m_pDebugOutput->append(tr(buf));
}

CPose::CPose()
{
    nIndex = 0;

    t[0] = 0.0;
    t[1] = 0.0;
    t[2] = 0.0;

    for (int i = 0; i<4; i++)
    {
        for (int j = 0; j<4; j++)
        {
            r[i][j]                 = 0.0;
            m[i * 4 + j]            = 0.0;
            globalMatrix[i * 4 + j] = 0.0;
        }
    }
}

CPose::~CPose()
{}

void CPose::SetGlobalMatrix(double *matrix)
{
    for (int i = 0; i<16; i++)
    {
        globalMatrix[i] = matrix[i];
    }
}

void CPose::SetMatrix(double *matrix)
{
    for (int i = 0; i<16; i++)
    {
        m[i] = matrix[i];
    }
}


void CPose::SetTranslation(double x, double y, double z)
{
    t[0] = x;
    t[1] = y;
    t[2] = z;
}

void CPose::SetRotation(double *rotation)
{
    r[0][0] = rotation[0];
    r[0][1] = rotation[1];
    r[0][2] = rotation[2];
    r[1][0] = rotation[3];
    r[1][1] = rotation[4];
    r[1][2] = rotation[5];
    r[2][0] = rotation[6];
    r[2][1] = rotation[7];
    r[2][2] = rotation[8];

    r[0][3] = 0;
    r[1][3] = 0;
    r[2][3] = 0;
    r[3][3] = 1.0;
}

void CPose::SetIndex(int i)
{
    nIndex = i;
}

void CPose::Save(QDataStream &out)
{
    out << nIndex;

    for (int i = 0; i<3; i++)
    {
        out << t[i];
    }

    for (int i = 0; i<16; i++)
    {
        out << globalMatrix[i];
    }

    for (int i = 0; i<16; i++)
    {
        out << m[i];
    }

    for (int i = 0; i<4; i++)
    {
        for (int j = 0; j<4; j++)
        {
            out << r[i][j];
        }
    }
}

void CPose::Load(QDataStream &in)
{
    in >> nIndex;

    for (int i = 0; i<3; i++)
    {
        in >> t[i];
    }

    for (int i = 0; i<16; i++)
    {
        in >> globalMatrix[i];
    }

    for (int i = 0; i<16; i++)
    {
        in >> m[i];
    }

    for (int i = 0; i<4; i++)
    {
        for (int j = 0; j<4; j++)
        {
            in >> r[i][j];
        }
    }
}