#ifndef VOMAINWINDOW_H
#define VOMAINWINDOW_H

#include <QObject>
#include <QMainWindow>
#include <QToolButton>
#include <QString>
#include <QSpinBox>
#include <QTimer>
#include <QLabel>
#include <QGridLayout>
#include <QComboBox>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <g2o/core/sparse_optimizer.h>
#include "TqcCamera.h"
#include "TqcEncodeDecode.h"
#include "TqcScene.h"
#include "OutputDialog.h"
#include "MatchDialog.h"
#include "TqcDebugControl.h"
#include "TqcBundleAdjust.h"

namespace Ui
{
class VOMainWindow;
}

class VOMainWindow : public QMainWindow
{
    Q_OBJECT

    typedef enum _CameraMode
    {
        Tracking_Cam_Mode,
        Record_Cam_Mode,
        Play_Video_File_Mode,
        Invalid_Cam_Mode
    } CameraMode;

public:
    explicit VOMainWindow(QWidget *parent = 0);
    ~VOMainWindow();

private:
    void                CreateAction();        // Create action for menu
    void                CreateBtnStyle();
    void                CreateMenu();
    void                CreateToolBar();
    void                CreateLayout();
    void                CreateStatusBar();
    void                CreateDocker();

    void                OpenCamera(int index, int FPS);
    void                CloseCamera();
    void                DebugOutput(QString str);
    void                DebugOutput(char *fmt, ...);
    void                DisplayCam(Mat &frame);
    void                CloseVideoFile();
    void                OpenVideoFile();
    void                ShowFrameNum();

    bool                GetPoseFromSeq(Mat &cur, Eigen::Isometry3d &pose);
    void                TrackingMode(Mat &frame);
    void                RecordMode(Mat &frame);
    bool                IsPause(); // Check if state is in pausing.
    void                ClearLastResult();
    void                GetPoseInformation();

    void                TrackingOutput(Eigen::Isometry3d pose);
    void                OutputDebugToOutputDialog(CPose *p);
    void                OutputDebugFromPose(CPose *p);

private slots:
    void                OpenCameraActionSlot();
    void                GetFrame();
    void                DrawMatches();

private:
    Ui::VOMainWindow *ui;
    QGridLayout      *m_gridLayout;
    QString          m_toolBtnStyle;
    QSpinBox         *m_cameraSpinBox;
    QToolButton      *m_toolPlayPauseBtn;
    QToolButton      *m_toolStopBtn;
    QToolButton      *m_toolRecordBtn;
    QToolButton      *m_toolPlayVideoBtn;
    QToolButton      *m_toolStepVideoBtn;
    QToolButton      *m_toolClearOutputBtn;
    QToolButton      *m_toolSaveOutputBtn;
    QSpinBox         *m_gotoFrameSpinBox;
    QToolButton      *m_toolGotoFrameBtn;
    QLabel           *m_totalFrameLabel;
    OutputDialog     m_outputDialog;
    MatchDialog      m_matchDialog;
    QLabel           *m_toolVertexLabel;
    QComboBox        *m_toolChannelComboBox;
    QSpinBox         *m_gotoVertexSpinBox;

    // Menu
    QAction *m_cameraOpenAction;                // Open camera action
    QAction *m_saveGraphFileAction;

    // Camera Control
    bool          m_bToolPlayVideo;
    bool          m_bToolRecord;
    bool          m_bToolPlay;
    bool          m_bCameraIsOpened;
    int           m_nCameraIndex;
    int           m_nCameraDevicesNum;
    CameraMode    m_camMode;
    QString       m_videoFileName;
    int           m_nVideoFileTotalFrameCount;
    int           m_nVideoCurFrameNum;
    CCamera       *m_pCamera;
    QTimer        *m_timer;
    Mat           m_prevFrame;
    CEncodeDecode m_encoder;
    int           m_nGotoFrame;

    // Select Vertex
    int m_nChannel;
    int m_nSelect;

    // Draw vertices
    Eigen::Isometry3d m_globalPose;
    int               m_nVerticsNum;
    double            m_dTotalTranslate[3];
    CScene            *m_pScene;

    // Debug output
    CDebugControl m_debugCtrl;

    // Bundle Adjustment Thread
    CBundleAdjustThread m_baThread;
    Mat                 m_pushMat;
    bool                m_bDrawMatch;

public slots:
    // menu
    void on_actionShow_Dock_triggered();
    void on_actionHide_Dock_triggered();
    void on_actionOpen_Camera_triggered();
    void on_actionSave_File_triggered();
    void on_actionLoad_File_triggered();

    // tool bar
    void on_toolPlayBtn_clicked();
    void on_toolRecordBtn_clicked();
    void on_toolPlayVideoBtn_clicked();
    void on_toolStepVideoBtn_clicked();
    void on_toolStopBtn_clicked();
    void on_toolClearOutputBtn_clicked();
    void on_toolClearOutputBtn_pressed();
    void on_toolGotoFrameBtn_clicked();

    void on_cameraSpinBox_changed(int index);
    void on_gotoFrameSpinBox_changed(int index);

    void on_gotoVertexSpinBox_changed(int index);
    void on_gotoVertexSpinBox_editingFinished();
    void on_toolChannelComboBox_changed(int index);

    // dock
    void on_debugDialogBtn_clicked();
    void on_matchDialogBtn_clicked();
    void on_showMatrixCheckBox_toggled(bool toggle);
    void on_showPoseTCheckBox_toggled(bool toggle);
    void on_showPoseRCheckBox_toggled(bool toggle);
    void on_showGlobalTCheckBox_toggled(bool toggle);
    void on_showGlobalRCheckBox_toggled(bool toggle);

    // track view
};
#endif // VOMAINWINDOW_H
