#include <QMenu>
#include <QDockWidget>
#include <QDebug>
#include <QSizePolicy>
#include <QFileDialog>
#include <opencv2/videoio/videoio_c.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/optimizable_graph.h>

#include "VoMainWindow.h"
#include "ui_vomainwindow.h"
#include "TqcCamera.h"
#include "TqcBundleAdjust.h"
#include "Tqc3dViewer.h"

VOMainWindow::VOMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::VOMainWindow),
    m_gridLayout(NULL),
    m_nVideoFileTotalFrameCount(0),
    m_nVideoCurFrameNum(0),
    m_pCamera(NULL),
    m_nGotoFrame(0),
    m_nVerticsNum(0)
{
    ui->setupUi(this);

    CreateBtnStyle();
    CreateAction();
    CreateToolBar();
    CreateLayout();
    CreateStatusBar();
    CreateDocker();

    // Debug Output view setting
    // ui->debugOutput->setFont();

    // QGLViewer
    ui->trackView->drawAxis(10);
    ui->trackView->drawGrid();
    ui->trackView->SetDebugDialog(&m_outputDialog);
    ui->trackView->SetDebugOutput(ui->debugOutput);
    // ui->trackView->setBackgroundColor(QColor(1.0, 0, 0));

    // QGLViewer signals
    // connect(ui->trackView, SIGNAL(SelectPose(int)), this, SLOT(on_SelectPose_triggered(int)));
    // connect(ui->trackView, &Viewer::SelectPose, this, &VOMainWindow::on_SelectPose_triggered);

    // menu signals
    // connect(ui->actionOpen_Camera, SIGNAL(triggered()), this, SLOT(on_actionOpen_Camera_triggered()));

    // Create Scene
    m_pScene = new CScene(ui->trackView);

    m_dTotalTranslate[0] = 0.0;
    m_dTotalTranslate[1] = 0.0;
    m_dTotalTranslate[2] = 0.0;

    m_globalPose.setIdentity();

    // Bundle Adjustment thread
    m_pushMat    = Mat();
    m_bDrawMatch = false;
}

VOMainWindow::~VOMainWindow()
{
    if (m_baThread.isRunning())
        m_baThread.stop();

    if (m_pCamera)
        delete m_pCamera;

    if (m_timer)
        delete m_timer;

    if (m_gridLayout)
        delete m_gridLayout;

    if (m_toolRecordBtn)
        delete m_toolRecordBtn;

    if (m_toolStopBtn)
        delete m_toolStopBtn;

    if (m_toolPlayPauseBtn)
        delete m_toolPlayPauseBtn;

    if (m_toolClearOutputBtn)
        delete m_toolClearOutputBtn;

    if (m_toolSaveOutputBtn)
        delete m_toolSaveOutputBtn;

    if (m_pScene)
        delete m_pScene;

    if (ui->trackView->m_graph)
        delete ui->trackView->m_graph;

    delete ui;
}

void VOMainWindow::on_actionShow_Dock_triggered()
{
    ui->dockWidget->show();
}

void VOMainWindow::on_actionHide_Dock_triggered()
{
    ui->dockWidget->hide();
}

void VOMainWindow::on_toolPlayBtn_clicked()
{
    if (m_camMode != Invalid_Cam_Mode &&
        m_camMode != Tracking_Cam_Mode)
        return;

    if (m_bToolPlay)
    {
        m_toolPlayPauseBtn->setText(tr("Play"));
        m_bToolPlay = false;
        m_baThread.Pause(true);
    }
    else
    {
        m_toolPlayPauseBtn->setText(tr("Pause"));
        m_bToolPlay = true;
        m_camMode   = Tracking_Cam_Mode;
        m_baThread.Pause(false);

        if (!m_bCameraIsOpened)
        {
            m_nVerticsNum = 0;
            OpenCamera(m_nCameraIndex, 30);
        }
    }
}

void VOMainWindow::on_toolRecordBtn_clicked()
{
    if (m_camMode != Invalid_Cam_Mode &&
        m_camMode != Record_Cam_Mode)
        return;

    if (m_bToolRecord)
    {
        m_toolRecordBtn->setText(tr("Record"));
        m_bToolRecord = false;
    }
    else
    {
        m_toolRecordBtn->setText(tr("Pause"));
        m_bToolRecord = true;
        m_camMode     = Record_Cam_Mode;

        if (!m_bCameraIsOpened)
        {
            m_videoFileName = QFileDialog::getSaveFileName(this, tr("打开录像文件"), "/User");
            std::string str       = m_videoFileName.toStdString();
            const char  *fileName = str.c_str();

            OpenCamera(m_nCameraIndex, 30);

            if (!m_encoder.EncodeInit(fileName, m_pCamera->m_nCamWidth, m_pCamera->m_nCamHeight))
            {
                qDebug() << "Cannot initialize encoder.";
                CloseCamera();
                m_camMode = Invalid_Cam_Mode;
                m_toolRecordBtn->setText(tr("Record"));
            }
        }
    }
}

void VOMainWindow::on_toolPlayVideoBtn_clicked()
{
    if (m_camMode != Invalid_Cam_Mode &&
        m_camMode != Play_Video_File_Mode)
        return;

    if (m_bToolPlayVideo)
    {
        m_toolPlayVideoBtn->setText(tr("Read"));
        m_bToolPlayVideo = false;
        m_baThread.Pause(true);
    }
    else
    {
        m_toolPlayVideoBtn->setText(tr("Pause"));
        m_bToolPlayVideo = true;
        m_camMode        = Play_Video_File_Mode;
        m_baThread.Pause(false);

        if (m_bCameraIsOpened == false)
        {
            m_nVerticsNum = 0;
            OpenVideoFile();
        }
    }
}

void VOMainWindow::on_toolStepVideoBtn_clicked()
{
    Mat frame;
    int nGotPic = 0;

    if (m_camMode != Play_Video_File_Mode)
        return;

    // Step to next frame.
    // It's enable only that video file doesn't play.
    if (m_bToolPlayVideo)
        return;

    int nSize = m_baThread.GetMatQueueSize();
    if (nSize == 0)
    {
        do
        {
            bool res = m_encoder.Decode(frame, nGotPic);
            if (!res)
            {
                CloseVideoFile();
                return;
            }
        }
        while (!nGotPic);

        ShowFrameNum();
        DisplayCam(frame);
        TrackingMode(frame);
    }

    nSize = m_baThread.GetMatQueueSize();
    m_baThread.Pause(false);

    while (nSize == m_baThread.GetMatQueueSize())
    {
        // do nothing. Busy waiting.
    }

    m_baThread.Pause(true);
}

void VOMainWindow::on_toolGotoFrameBtn_clicked()
{
    if (m_camMode != Play_Video_File_Mode)
        return;

    if (m_bToolPlayVideo)
        return;

    m_toolPlayVideoBtn->setText(tr("Pause"));
    m_bToolPlayVideo = true;
}

void VOMainWindow::on_toolStopBtn_clicked()
{
    switch (m_camMode)
    {
    case Play_Video_File_Mode:
        CloseVideoFile();
        break;

    case Tracking_Cam_Mode:
        CloseCamera();
        break;

    case Record_Cam_Mode:
        CloseCamera();
        break;

    default:
        break;
    }
}

void VOMainWindow::on_toolClearOutputBtn_clicked()
{
    ui->debugOutput->clear();
}

void VOMainWindow::on_toolClearOutputBtn_pressed()
{}

void VOMainWindow::on_actionOpen_Camera_triggered()
{
    qDebug() << "Open Camera";
}

void VOMainWindow::on_actionSave_File_triggered()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("保存图文件"), "/User");

#if 0
    SparseOptimizer *graph = ui->trackView->m_graph;
    graph->save(fileName.toStdString().c_str());
#endif

    ui->trackView->Save(fileName.toStdString());
}

void VOMainWindow::on_actionLoad_File_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("加载图文件"), "/User");

#if 0
    SparseOptimizer *graph = ui->trackView->m_graph;
    graph->clear();
    graph->load(fileName.toStdString().c_str());
#endif

    ui->trackView->Load(fileName.toStdString());
}

void VOMainWindow::CreateAction()
{
    // 创建打开文件动作
    m_cameraOpenAction = new QAction(tr("打开摄像头"), this);

    // 设置打开文件动作的提示信息
    m_cameraOpenAction->setStatusTip(tr("打开一个指定的摄像头"));

    // 关联打开文件动作的信号和槽
    connect(m_cameraOpenAction, SIGNAL(triggered()), this, SLOT(OpenCameraActionSlot()));
}

void VOMainWindow::CreateBtnStyle()
{
    // Button style
    m_toolBtnStyle = "QToolButton{\
    color:rgb(255, 255, 255);\
    min-height:20;\
    border-style:solid;\
    border-top-left-radius:2px;\
    border-top-right-radius:2px;\
    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop:0 rgb(226,236,241),\
    stop: 0.3 rgb(160,160,160),\
    stop: 1 rgb(140,140,140));\
    border:1px;\
    border-radius:5px;padding:2px 4px;/*border-radius控制圆角大小*/\
    }\
    QToolButton:hover{ /*鼠标放上后*/\
    color:rgb(255, 255, 255);\
    min-height:20;\
    border-style:solid;\
    border-top-left-radius:2px;\
    border-top-right-radius:2px;\
    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop:0 rgb(226,236,241),\
    stop: 0.3 rgb(160,160,160),\
    stop: 1 rgb(120,120,120));\
    border:1px;\
    border-radius:5px;padding:2px 4px;\
    }\
    QToolButton:pressed{ /*按下按钮后*/\
    color:rgb(255, 255, 255);\
    min-height:20;\
    border-style:solid;\
    border-top-left-radius:2px;\
    border-top-right-radius:2px;\
    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop:0 rgb(226,236,241),\
    stop: 0.3 rgb(190,190,190),\
    stop: 1 rgb(160,160,160));\
    border:1px;\
    border-radius:5px;padding:2px 4px;\
    }\
    QToolButton:checked{ /*选中后*/\
    color:rgb(255, 255, 255);\
    min-height:20;\
    border-style:solid;\
    border-top-left-radius:2px;\
    border-top-right-radius:2px;\
    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop:0 rgb(226,236,241),\
    stop: 0.3 rgb(190,190,190),\
    stop: 1 rgb(160,160,160));\
    border:1px;\
    border-radius:5px;padding:2px 4px;\
    }\
    ";
}

void VOMainWindow::CreateMenu()
{
    ui->menuFile->addAction(m_cameraOpenAction);
}

void VOMainWindow::CreateLayout()
{
    // m_gridLayout = new QGridLayout;

//    ui->cameraOutput->setMinimumSize(0, 0);
//    ui->trackView->setMinimumSize(0, 0);
//    ui->debugOutput->setMinimumSize(0, 0);
//    ui->shellInput->setMinimumSize(0, 0);
//
//    ui->cameraOutput->setMaximumSize(0x7fffffff, 0x7fffffff);
//    ui->trackView->setMaximumSize(0x7fffffff, 0x7fffffff);
//    ui->debugOutput->setMaximumSize(0x7fffffff, 0x7fffffff);
//    ui->shellInput->setMaximumSize(0x7fffffff, 0x7fffffff);
//
//    ui->cameraOutput->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    ui->trackView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    ui->debugOutput->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    ui->shellInput->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
//    ui->centralWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

//    m_gridLayout->addWidget(ui->cameraOutput, 0, 0, 1, 1);
//    m_gridLayout->addWidget(ui->trackView, 0, 1, 2, 3);
//    m_gridLayout->addWidget(ui->shellInput, 1, 0, 2, 1);
//    m_gridLayout->addWidget(ui->debugOutput, 1, 1, 1, 3);
//
//    //ui->centralWidget->setLayout(m_gridLayout);
//    setLayout(m_gridLayout);
}

void VOMainWindow::CreateToolBar()
{
    m_timer   = NULL;
    m_pCamera = NULL;

    // Camera Control tool bar
    QLabel *cameraSpinBoxLabel = new QLabel;
    cameraSpinBoxLabel->setText(tr("Camera"));
    ui->mainToolBar->addWidget(cameraSpinBoxLabel);

    m_cameraSpinBox     = new QSpinBox(this);
    m_nCameraDevicesNum = GetCameraDeviceCount();
    DebugOutput((char*)"Camera devices' num: %d", m_nCameraDevicesNum);
    m_cameraSpinBox->setRange(0, m_nCameraDevicesNum - 1);
    m_cameraSpinBox->setValue(m_nCameraDevicesNum - 1);
    ui->mainToolBar->addWidget(m_cameraSpinBox);
    m_nCameraIndex = m_nCameraDevicesNum - 1;

    m_toolPlayPauseBtn = new QToolButton(this);
    m_toolPlayPauseBtn->setText(tr("Play"));
    m_bToolPlay       = false;
    m_bCameraIsOpened = false;
    m_toolPlayPauseBtn->setCheckable(true);
    m_toolPlayPauseBtn->setStyleSheet(m_toolBtnStyle);

    m_toolStopBtn = new QToolButton(this);
    m_toolStopBtn->setText(tr("Stop"));
    m_toolStopBtn->setStyleSheet(m_toolBtnStyle);

    m_toolRecordBtn = new QToolButton(this);
    m_toolRecordBtn->setText(tr("Record"));
    m_toolRecordBtn->setStyleSheet(m_toolBtnStyle);
    m_bToolRecord = false;

    m_toolPlayVideoBtn = new QToolButton(this);
    m_toolPlayVideoBtn->setText(tr("Read"));
    m_toolPlayVideoBtn->setStyleSheet(m_toolBtnStyle);
    m_bToolPlayVideo = false;

    m_toolStepVideoBtn = new QToolButton(this);
    m_toolStepVideoBtn->setText(tr("Step"));
    m_toolStepVideoBtn->setStyleSheet(m_toolBtnStyle);

    m_gotoFrameSpinBox = new QSpinBox(this);
    m_gotoFrameSpinBox->setRange(0, m_nVideoFileTotalFrameCount);
    m_gotoFrameSpinBox->setValue(0);

    m_toolGotoFrameBtn = new QToolButton(this);
    m_toolGotoFrameBtn->setText(tr("Go"));
    m_toolGotoFrameBtn->setStyleSheet(m_toolBtnStyle);

    m_camMode = Invalid_Cam_Mode;
    ui->mainToolBar->addWidget(m_toolPlayPauseBtn);
    ui->mainToolBar->addWidget(m_toolStopBtn);
    ui->mainToolBar->addWidget(m_toolRecordBtn);
    ui->mainToolBar->addWidget(m_toolPlayVideoBtn);
    ui->mainToolBar->addWidget(m_toolStepVideoBtn);
    ui->mainToolBar->addWidget(m_gotoFrameSpinBox);
    ui->mainToolBar->addWidget(m_toolGotoFrameBtn);

    ui->mainToolBar->addSeparator();

    // Debug output control tool bar
    m_toolClearOutputBtn = new QToolButton(this);
    m_toolSaveOutputBtn  = new QToolButton(this);

    m_toolClearOutputBtn->setText(tr("Clear"));
    m_toolSaveOutputBtn->setText(tr("Save"));

    m_toolClearOutputBtn->setStyleSheet(m_toolBtnStyle);
    m_toolSaveOutputBtn->setStyleSheet(m_toolBtnStyle);

    ui->mainToolBar->addWidget(m_toolClearOutputBtn);
    ui->mainToolBar->addWidget(m_toolSaveOutputBtn);

    // Separate
    ui->mainToolBar->addSeparator();

    // Channel and vertex select
    m_toolVertexLabel = new QLabel(this);
    m_toolVertexLabel->setText(tr("Vertex:"));
    m_nSelect = -1;

    m_toolChannelComboBox = new QComboBox(this);
    m_toolChannelComboBox->addItem(tr("Video"));

    for (int i = 0; i<TQC_LOAD_POSES_ARRAY_NUM; i++)
    {
        char buf[16];

        memset(buf, 0, 16);
        sprintf(buf, "Load%d", i + 1);
        m_toolChannelComboBox->addItem(tr(buf));
    }

    m_nChannel = -1;

    m_gotoVertexSpinBox = new QSpinBox(this);
    m_gotoVertexSpinBox->setRange(1, 1);
    m_gotoVertexSpinBox->setValue(1);

    ui->mainToolBar->addWidget(m_toolVertexLabel);
    ui->mainToolBar->addWidget(m_toolChannelComboBox);
    ui->mainToolBar->addWidget(m_gotoVertexSpinBox);
    ui->mainToolBar->addSeparator();

    // tool bar signal and SLOT connect.
    connect(m_toolPlayPauseBtn, SIGNAL(clicked()), this, SLOT(on_toolPlayBtn_clicked()));
    connect(m_toolClearOutputBtn, SIGNAL(clicked()), this, SLOT(on_toolClearOutputBtn_clicked()));
    connect(m_toolClearOutputBtn, SIGNAL(pressed()), this, SLOT(on_toolClearOutputBtn_pressed()));
    connect(m_cameraSpinBox, SIGNAL(valueChanged(int)), this, SLOT(on_cameraSpinBox_changed(int)));
    connect(m_toolRecordBtn, SIGNAL(clicked()), this, SLOT(on_toolRecordBtn_clicked()));
    connect(m_toolStopBtn, SIGNAL(clicked()), this, SLOT(on_toolStopBtn_clicked()));
    connect(m_toolPlayVideoBtn, SIGNAL(clicked()), this, SLOT(on_toolPlayVideoBtn_clicked()));
    connect(m_toolStepVideoBtn, SIGNAL(clicked()), this, SLOT(on_toolStepVideoBtn_clicked()));
    connect(m_toolGotoFrameBtn, SIGNAL(clicked()), this, SLOT(on_toolGotoFrameBtn_clicked()));
    connect(m_gotoFrameSpinBox, SIGNAL(valueChanged(int)), this, SLOT(on_gotoFrameSpinBox_changed(int)));

    connect(m_gotoVertexSpinBox, SIGNAL(valueChanged(int)), this, SLOT(on_gotoVertexSpinBox_changed(int)));
    connect(m_toolChannelComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(on_toolChannelComboBox_changed(int)));
    connect(m_gotoVertexSpinBox, SIGNAL(editingFinished()), this, SLOT(on_gotoVertexSpinBox_editingFinished()));
}

void VOMainWindow::CreateStatusBar()
{
    // Label for total frame number
    m_totalFrameLabel = new QLabel;
    m_totalFrameLabel->setText(tr("Total: 0  Current: 0"));
    ui->statusBar->addWidget(m_totalFrameLabel);
}

void VOMainWindow::CreateDocker()
{
    connect(ui->debugDialogBtn1, SIGNAL(clicked()), this, SLOT(on_debugDialogBtn_clicked()));
    connect(ui->showMatrix, SIGNAL(toggled(bool)), this, SLOT(on_showMatrixCheckBox_toggled(bool)));
    connect(ui->showPoseT, SIGNAL(toggled(bool)), this, SLOT(on_showPoseTCheckBox_toggled(bool)));
    connect(ui->showPoseR, SIGNAL(toggled(bool)), this, SLOT(on_showPoseRCheckBox_toggled(bool)));
    connect(ui->showGlobalT, SIGNAL(toggled(bool)), this, SLOT(on_showGlobalTCheckBox_toggled(bool)));
    connect(ui->showGlobalR, SIGNAL(toggled(bool)), this, SLOT(on_showGlobalRCheckBox_toggled(bool)));

    // Set debug control
    ui->showMatrix->setChecked(m_debugCtrl.m_bMatrixToggle);
    ui->showPoseT->setChecked(m_debugCtrl.m_bPoseTToggle);
    ui->showPoseR->setChecked(m_debugCtrl.m_bPoseRToggle);
    ui->showGlobalT->setChecked(m_debugCtrl.m_bGlobalTToggle);
    ui->showGlobalR->setChecked(m_debugCtrl.m_bGlobalRToggle);
}

void VOMainWindow::ClearLastResult()
{
    // Clear last result
    m_globalPose.setIdentity();
    ui->trackView->Release();
    m_prevFrame = Mat();

    // Create grapher for pose
    ui->trackView->m_graph = new SparseOptimizer();
    if (ui->trackView->m_graph)
    {
        g2o::VertexSE3 *v = new g2o::VertexSE3();
        v->setId(0);
        v->setFixed(true);   // 第一个点固定为零

        // 预设值为单位Pose，因为我们不知道任何信息
        v->setEstimate(g2o::SE3Quat());
        ui->trackView->m_graph->addVertex(v);
    }
}

void VOMainWindow::OpenCamera(int index, int FPS)
{
    int nDelay = 1000 / FPS;

    if (m_pCamera)
    {
        delete m_pCamera;
    }

    if (m_camMode == Tracking_Cam_Mode)
    {
        ClearLastResult();
    }

    m_bCameraIsOpened = true;
    m_pCamera         = new CCamera(index);
    m_pCamera->OpenCamera();
    m_timer = new QTimer(this);
    m_timer->start(nDelay);

    connect(m_timer, SIGNAL(timeout()), this, SLOT(GetFrame()));

    DebugOutput((char*)"Open camera %d", index);
    DebugOutput((char*)"Start timer, the delay is %d", nDelay);

    // Start BA thread to process frame.
    m_baThread.start();
}

void VOMainWindow::CloseCamera()
{
    if (m_camMode == Record_Cam_Mode)
    {
        m_encoder.EncodeRelease();
        m_bToolRecord = false;
        m_toolRecordBtn->setText(tr("Record"));
    }

    if (m_camMode == Tracking_Cam_Mode)
    {
        // Close BA thread.
        m_baThread.stop();

        m_bToolPlay = false;
        m_toolPlayPauseBtn->setText(tr("Play"));
    }

    if (m_pCamera)
    {
        delete m_pCamera;
        m_pCamera = NULL;
    }

    m_bCameraIsOpened = false;

    if (m_timer)
    {
        m_timer->stop();
        delete m_timer;
        m_timer = NULL;
    }

    m_camMode           = Invalid_Cam_Mode;
    m_nVideoCurFrameNum = 0;
}

void VOMainWindow::OpenVideoFile()
{
    m_videoFileName = QFileDialog::getOpenFileName(this, tr("打开录像文件"), "/User");
    std::string str = m_videoFileName.toStdString();

    DebugOutput((char*)"Open file: %s", str.c_str());
    if (!m_encoder.DecodeInit(str.c_str()))
    {
        DebugOutput((char*)"Cannot initialize decoder");
        CloseVideoFile();
        return;
    }

    ClearLastResult();

    int nDelay = 33;
    m_timer = new QTimer(this);
    m_timer->start(nDelay);

    connect(m_timer, SIGNAL(timeout()), this, SLOT(GetFrame()));

    m_bCameraIsOpened = true;
    m_camMode         = Play_Video_File_Mode;

    // Show total frame
    m_nVideoFileTotalFrameCount = m_encoder.GetTotalFrameNum();
    ShowFrameNum();
    m_gotoFrameSpinBox->setRange(0, m_nVideoFileTotalFrameCount);

    // Start BA thread to process frame.
    m_baThread.start();
}

void VOMainWindow::CloseVideoFile()
{
    m_bToolPlayVideo = false;

    // Close BA thread.
    m_baThread.stop();

    m_encoder.DecodeRelease();
    if (m_timer)
    {
        m_timer->stop();
        delete m_timer;
        m_timer = NULL;
    }

    m_bCameraIsOpened = false;

    m_toolPlayVideoBtn->setText(tr("Read"));
    m_camMode           = Invalid_Cam_Mode;
    m_nVideoCurFrameNum = 0;
}

void VOMainWindow::ShowFrameNum()
{
    char buf[256];

    // Video frame counter.
    m_nVideoCurFrameNum++;

    memset(buf, 0, 256);
    sprintf(buf, "Total: %d   Current: %d", m_nVideoFileTotalFrameCount, m_nVideoCurFrameNum);
    m_totalFrameLabel->setText(tr(buf));
}

void VOMainWindow::DisplayCam(Mat &frame)
{
    QImage image   = QImage(frame.data, frame.cols, frame.rows, QImage::Format_RGB888).rgbSwapped();
    int    imageW  = image.width();
    int    imageH  = image.height();
    int    outputW = ui->cameraOutput->width();
    int    outputH = ui->cameraOutput->height();
    float  fScale  = (float)outputW / (float)imageW;

    outputH = (int)(fScale * (float)imageH);

    image = image.scaled(outputW, outputH);
    ui->cameraOutput->setPixmap(QPixmap::fromImage(image));
}

void VOMainWindow::OutputDebugToOutputDialog(CPose *p)
{
    m_outputDialog.LoadInformFromPose(TQC_VIDEO_POSE_ARRAY_INDEX, p);
}

void VOMainWindow::TrackingOutput(Eigen::Isometry3d pose)
{
    char                                buf[256];
    Eigen::Isometry3d::TranslationPart  t = pose.translation();
    Eigen::Isometry3d::LinearMatrixType r = pose.rotation();
    Eigen::Isometry3d::MatrixType       a = pose.matrix();

    if (m_debugCtrl.m_bMatrixToggle)
    {
        m_debugCtrl.OutputMatrix(buf, 256, m_nVerticsNum, pose);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bPoseTToggle)
    {
        m_debugCtrl.OutputPoseT(buf, 256, m_nVerticsNum, t);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bPoseRToggle)
    {
        m_debugCtrl.OutputPoseR(buf, 256, m_nVerticsNum, r);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bGlobalTToggle)
    {
        m_debugCtrl.OutputGlobalT(buf, 256, m_nVerticsNum, m_globalPose);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bGlobalRToggle)
    {
        m_debugCtrl.OutputGlobalR(buf, 256, m_nVerticsNum, m_globalPose);
        DebugOutput(buf);
    }
}

void VOMainWindow::GetPoseInformation()
{
    Eigen::Isometry3d pose;

    // Fetch pose information
    if (!m_baThread.PopPose(pose))
    {
        return;
    }

    m_nVerticsNum++;

    Eigen::Isometry3d::TranslationPart  t = pose.translation();
    Eigen::Isometry3d::LinearMatrixType r = pose.rotation();
    Eigen::Isometry3d::MatrixType       a = pose.matrix();

    m_globalPose = m_globalPose * a;

    // Print matrix/pose/global pose to output view.
    TrackingOutput(pose);

    // Add vertex to grapher
    SparseOptimizer *graph = ui->trackView->m_graph;
    g2o::VertexSE3  *v     = new g2o::VertexSE3();
    CPose           *p     = new CPose;

    v->setId(m_nVerticsNum);

    // 由于深度不知道，只能把深度设置为10了
    double z = t(2) * TQC_Z_SCALE;  // We set scale as 10.
    double x = (t(0) - CCamera::cx) * z / CCamera::fx;
    double y = (t(1) - CCamera::cy) * z / CCamera::fy;

    m_dTotalTranslate[0] += x;
    m_dTotalTranslate[1] += y;
    m_dTotalTranslate[2] += z;

    v->setMarginalized(true);
    // v->setEstimate(Eigen::Vector3d(x, y, z));
    // double data[7] = {m_dTotalTranslate[0], m_dTotalTranslate[1], m_dTotalTranslate[2], 0.0, 0.0, 0.0, 1.0};
    double data[7] = {x, y, z, 0.0, 0.0, 0.0, 1.0};
    v->setEstimateDataImpl(data);
    // v->setUserData(<#g2o::HyperGraph::Data *obs#>)
    graph->addVertex(v);

    // 由于深度不知道，只能把深度设置为10了
    /*z = m_globalPose(2, 3) * TQC_Z_SCALE;  // We set scale as 10.
       x = (m_globalPose(0, 3) - CCamera::cx) * z / CCamera::fx;
       y = (m_globalPose(2, 3) - CCamera::cy) * z / CCamera::fy;*/

    double rotation[9] = {r(0, 0), r(0, 1), r(0, 2),
                          r(1, 0), r(1, 1), r(1, 2),
                          r(2, 0), r(2, 1), r(2, 2)};
    double globalMat[16] = {m_globalPose(0, 0), m_globalPose(0, 1), m_globalPose(0, 2), m_globalPose(0, 3),
                            m_globalPose(1, 0), m_globalPose(1, 1), m_globalPose(1, 2), m_globalPose(1, 3),
                            m_globalPose(2, 0), m_globalPose(2, 1), m_globalPose(2, 2), m_globalPose(2, 3),
                            m_globalPose(3, 0), m_globalPose(3, 1), m_globalPose(3, 2), m_globalPose(3, 3)};
    double mat[16] = {pose(0, 0), pose(0, 1), pose(0, 2), pose(0, 3),
                      pose(1, 0), pose(1, 1), pose(1, 2), pose(1, 3),
                      pose(2, 0), pose(2, 1), pose(2, 2), pose(2, 3),
                      pose(3, 0), pose(3, 1), pose(3, 2), pose(3, 3)};

    p->SetIndex(m_nVerticsNum);
    p->SetTranslation(t(0), t(1), t(2));
    p->SetRotation(rotation);
    p->SetGlobalMatrix(globalMat);
    p->SetMatrix(mat);
    ui->trackView->m_poses.push_back(p);

    // Print matrix/pose/global pose to output dialog, if it is shown.
    OutputDebugToOutputDialog(p);

    g2o::EdgeSE3Expmap *edge = new g2o::EdgeSE3Expmap();
    edge->setVertex(0, dynamic_cast<g2o::VertexSE3Expmap*>(graph->vertex(m_nVerticsNum - 1)));
    edge->setVertex(1, dynamic_cast<g2o::VertexSE3Expmap*>(graph->vertex(m_nVerticsNum)));
    edge->setParameterId(0, 0);
    if (!graph->addEdge(edge))
    {
        qDebug() << "Cannot add edge";
    }

    ui->trackView->SetUpdateDisplay(true);
    ui->trackView->updateGL();
}

void VOMainWindow::TrackingMode(Mat &frame)
{
    Eigen::Isometry3d pose;
    bool              res = false;

    // If frame is not null, push it to queue.
    if (!frame.empty())
    {
        // Push new Mat to Queue.
        res = m_baThread.PushMat(frame);

        // Mat Queue is full.
        if (!res)
        {
            m_pushMat = frame;
        }
    }
}

void VOMainWindow::RecordMode(Mat &frame)
{
    // Convert frame to YUV.
    Mat yuv;

    if (frame.empty())
    {
        qDebug() << __FUNCTION__ << "(" << __LINE__ << "): frame is empty";
        return;
    }

    cvtColor(frame, yuv, CV_RGB2YUV_I420);

    m_encoder.EncodeFrame(yuv);
}

void VOMainWindow::DrawMatches()
{
    if (m_bDrawMatch)
    {
        Mat drawMatch;
        Mat drawGoodMatch;
        
        m_baThread.GetDrawMatchMat(drawMatch, drawGoodMatch);
        m_matchDialog.DrawMatch(drawMatch, drawGoodMatch);
    }
}

bool VOMainWindow::IsPause()
{
    if (m_camMode == Record_Cam_Mode && !m_bToolRecord)
        return true;

    // If camera or video playing is paused, we should clear mat queue.
    if (((m_camMode == Tracking_Cam_Mode) && !m_bToolPlay) ||
        ((m_camMode == Play_Video_File_Mode) && !m_bToolPlayVideo))
    {
        // This code for step mode.
        GetPoseInformation();

        // Show match mat on MatchDialog.
        DrawMatches();

        return true;
    }

    return false;
}

void VOMainWindow::GetFrame()
{
    Mat frame = Mat();

    // Stop state, we should do nothing.
    if (m_camMode == Invalid_Cam_Mode)
        return;

    // Pause state, we will clear some queue.
    if (IsPause())
        return;

    // If there is the last frame that isn't pushed to queue,
    // we will fetch frame from camera or video file. We just
    // push the last frame to queue.
    if (m_pushMat.empty())
    {
        if (m_camMode == Play_Video_File_Mode)
        {
            int nGotPic = 0;

            do
            {
                bool res = m_encoder.Decode(frame, nGotPic);
                if (!res)
                {
                    CloseVideoFile();
                    return;
                }
            }
            while (!nGotPic);

            ShowFrameNum();
        }
        else
        {
            m_pCamera->ReadCamera(frame);
        }
    }
    else
    {
        frame     = m_pushMat;
        m_pushMat = Mat();
    }

    DisplayCam(frame);

    switch (m_camMode)
    {
    case Play_Video_File_Mode:
    case Tracking_Cam_Mode:
        TrackingMode(frame);
        break;

    case Record_Cam_Mode:
        RecordMode(frame);
        break;

    default:
        break;
    }

    GetPoseInformation();

    // Show match mat on MatchDialog.
    DrawMatches();

    // Stop video file playing.
    if (m_nGotoFrame != 0 && m_nVideoCurFrameNum == m_nGotoFrame)
    {
        m_toolPlayVideoBtn->setText(tr("Read"));
        m_bToolPlayVideo = false;
    }
}

void VOMainWindow::OpenCameraActionSlot()
{
    qDebug() << "Open Camera";
}

void VOMainWindow::DebugOutput(QString str)
{
    ui->debugOutput->append(str);
}

void VOMainWindow::DebugOutput(char *fmt, ...)
{
    char    buf[256];
    va_list arg_ptr;

    va_start(arg_ptr, fmt);
    memset(buf, 0, 256);
    vsprintf(buf, fmt, arg_ptr);
    va_end(arg_ptr);

    ui->debugOutput->append(tr(buf));
}

void VOMainWindow::on_cameraSpinBox_changed(int index)
{
    m_nCameraIndex = index;
}

void VOMainWindow::on_gotoFrameSpinBox_changed(int index)
{
    m_nGotoFrame = index;
}

bool VOMainWindow::GetPoseFromSeq(Mat &cur, Eigen::Isometry3d &pose)
{
    Mat match;
    Mat goodMatch;

    return PoseEstimate(m_prevFrame, cur, pose, m_bDrawMatch, match, goodMatch);
}

void VOMainWindow::on_debugDialogBtn_clicked()
{
    if (m_outputDialog.m_bShown)
    {
        m_outputDialog.hide();
    }
    else
    {
        m_outputDialog.show();
    }

    m_outputDialog.m_bShown = !m_outputDialog.m_bShown;
}

void VOMainWindow::on_matchDialogBtn_clicked()
{
    if (m_matchDialog.isHidden())
    {
        m_matchDialog.show();
        m_bDrawMatch = true;
    }
    else
    {
        m_matchDialog.hide();
        m_bDrawMatch = false;
    }

    m_baThread.SetDrawMatch(m_bDrawMatch);
}

void VOMainWindow::OutputDebugFromPose(CPose *p)
{
    char buf[256];

    if (m_debugCtrl.m_bMatrixToggle)
    {
        m_debugCtrl.OutputMatrix(buf, 256, p->nIndex, p->m);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bPoseTToggle)
    {
        m_debugCtrl.OutputPoseT(buf, 256, p->nIndex, p->t);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bPoseRToggle)
    {
        m_debugCtrl.OutputPoseR(buf, 256, p->nIndex, (double*)p->r[0]);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bGlobalTToggle)
    {
        m_debugCtrl.OutputGlobalT(buf, 256, p->nIndex, p->globalMatrix);
        DebugOutput(buf);
    }

    if (m_debugCtrl.m_bGlobalRToggle)
    {
        m_debugCtrl.OutputGlobalR(buf, 256, p->nIndex, p->globalMatrix);
        DebugOutput(buf);
    }
}

void VOMainWindow::on_showMatrixCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bMatrixToggle = toggle;
}

void VOMainWindow::on_showPoseTCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bPoseTToggle = toggle;
}

void VOMainWindow::on_showPoseRCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bPoseRToggle = toggle;
}

void VOMainWindow::on_showGlobalTCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bGlobalTToggle = toggle;
}

void VOMainWindow::on_showGlobalRCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bGlobalRToggle = toggle;
}

void VOMainWindow::on_gotoVertexSpinBox_changed(int index)
{
    m_nSelect = index;
}

void VOMainWindow::on_gotoVertexSpinBox_editingFinished()
{
    PoseArray *pPoses = NULL;
    CPose     *p      = NULL;

    if (m_nChannel == 0)
    {
        pPoses = &ui->trackView->m_poses;
    }
    else
    {
        pPoses = &ui->trackView->m_loadPoseArray[m_nChannel - 1];
    }

    p = (*pPoses)[m_nSelect - 1];

    OutputDebugFromPose(p);
}

void VOMainWindow::on_toolChannelComboBox_changed(int index)
{
    int nVertexSize = 0;

    m_nChannel = index;

    if (index == 0)
    {
        nVertexSize = ui->trackView->m_poses.size();
    }
    else
    {
        nVertexSize = ui->trackView->m_loadPoseArray[index - 1].size();
    }

    m_gotoVertexSpinBox->setRange(1, nVertexSize);

    // Show total vertex numbers
    DebugOutput((char*)"Channel: %d\n"
                "Total Vertex: %d",
                m_nChannel, nVertexSize);
}
