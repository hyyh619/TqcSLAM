#include <QString>
#include <QFileDialog>
#include <QTextStream>

#include "OutputDialog.h"
#include "ui_OutputDialog.h"

OutputDialog::OutputDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::OutputDialog)
{
    ui->setupUi(this);

    m_nSelectStr  = 0;
    m_nSelectStr2 = 0;

    connect(ui->showMatrix, SIGNAL(toggled(bool)), this, SLOT(on_showMatrixCheckBox_toggled(bool)));
    connect(ui->showPoseT, SIGNAL(toggled(bool)), this, SLOT(on_showPoseTCheckBox_toggled(bool)));
    connect(ui->showPoseR, SIGNAL(toggled(bool)), this, SLOT(on_showPoseRCheckBox_toggled(bool)));
    connect(ui->showGlobalT, SIGNAL(toggled(bool)), this, SLOT(on_showGlobalTCheckBox_toggled(bool)));
    connect(ui->showGlobalR, SIGNAL(toggled(bool)), this, SLOT(on_showGlobalRCheckBox_toggled(bool)));

    // Set debug control
    ui->showPoseT->setChecked(m_debugCtrl.m_bPoseTToggle);
    ui->showPoseR->setChecked(m_debugCtrl.m_bPoseRToggle);
    ui->showMatrix->setChecked(m_debugCtrl.m_bMatrixToggle);
    ui->showGlobalT->setChecked(m_debugCtrl.m_bGlobalTToggle);
    ui->showGlobalR->setChecked(m_debugCtrl.m_bGlobalRToggle);

    m_bShown = false;

    for (int i = 0; i<TQC_TOTAL_POSES_ARRAY_NUM; i++)
    {
        m_savedStr[i].clear();
    }

    // Add channel select combo box items
    ui->channelComboBox->addItem(tr("Video"));
    ui->channelComboBox2->addItem(tr("Video"));

    for (int i = 0; i<TQC_LOAD_POSES_ARRAY_NUM; i++)
    {
        char buf[32];

        memset(buf, 0, 32);
        sprintf(buf, "Load%d", i + 1);
        ui->channelComboBox->addItem(tr(buf));
        ui->channelComboBox2->addItem(tr(buf));
    }
}

OutputDialog::~OutputDialog()
{
    delete ui;

    for (int i = 0; i<TQC_TOTAL_POSES_ARRAY_NUM; i++)
    {
        ClearString(i);
    }
}

void OutputDialog::UpdateChannelName(int nLoad, QString name)
{
    ui->channelComboBox->setItemText(nLoad, name);
    ui->channelComboBox2->setItemText(nLoad, name);
}

QTextBrowser* OutputDialog::GetView(int nView)
{
    QTextBrowser *pView;

    switch (nView)
    {
    case 1:
        pView = ui->outputView;
        break;

    case 2:
        pView = ui->outputView2;
        break;

    default:
        return NULL;
    }

    return pView;
}

void OutputDialog::UpdateDisplay(int nView, int nSelect)
{
    QTextBrowser *pView = GetView(nView);
    StringArray  *pArr  = &m_savedStr[nSelect];

    pView->clear();

    for (StringArray::iterator it = pArr->begin(); it != pArr->end(); it++)
    {
        QString *p = *it;

        if (m_debugCtrl.m_bMatrixToggle)
        {
            if (p->contains(tr(TQC_DEBUG_STR_MATRIX)))
            {
                pView->append(*p);
            }
        }

        if (m_debugCtrl.m_bPoseTToggle)
        {
            if (p->contains(tr(TQC_DEBUG_STR_TRANSLATION)))
            {
                pView->append(*p);
            }
        }

        if (m_debugCtrl.m_bPoseRToggle)
        {
            if (p->contains(tr(TQC_DEBUG_STR_ROTATION)))
            {
                pView->append(*p);
            }
        }

        if (m_debugCtrl.m_bGlobalTToggle)
        {
            if (p->contains(tr(TQC_DEBUG_STR_GLOBAL_T)))
            {
                pView->append(*p);
            }
        }

        if (m_debugCtrl.m_bGlobalRToggle)
        {
            if (p->contains(tr(TQC_DEBUG_STR_GLOBAL_R)))
            {
                pView->append(*p);
            }
        }
    }
}

void OutputDialog::LoadInformFromPose(int nLoad, CPose *p)
{
    char buf[256];

    m_debugCtrl.OutputMatrix(buf, 256, p->nIndex, p->m);
    SaveString(nLoad, buf);

    m_debugCtrl.OutputPoseT(buf, 256, p->nIndex, p->t);
    SaveString(nLoad, buf);
    m_debugCtrl.OutputPoseR(buf, 256, p->nIndex, (double*)p->r[0]);
    SaveString(nLoad, buf);

    m_debugCtrl.OutputGlobalT(buf, 256, p->nIndex, p->globalMatrix);
    SaveString(nLoad, buf);
    m_debugCtrl.OutputGlobalR(buf, 256, p->nIndex, p->globalMatrix);
    SaveString(nLoad, buf);
}

void OutputDialog::UpdateAllDisplay()
{
    UpdateDisplay(1, m_nSelectStr);
    UpdateDisplay(2, m_nSelectStr2);
}

void OutputDialog::on_channelComboBox_currentIndexChanged(int index)
{
    if (index < 0 || index >= TQC_TOTAL_POSES_ARRAY_NUM)
        return;

    m_nSelectStr = index;

    UpdateDisplay(1, m_nSelectStr);
}

void OutputDialog::on_channelComboBox2_currentIndexChanged(int index)
{
    if (index < 0 || index >= TQC_TOTAL_POSES_ARRAY_NUM)
        return;

    m_nSelectStr2 = index;

    UpdateDisplay(2, m_nSelectStr2);
}

void OutputDialog::on_showMatrixCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bMatrixToggle = toggle;
    UpdateAllDisplay();
}

void OutputDialog::on_showPoseTCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bPoseTToggle = toggle;
    UpdateAllDisplay();
}

void OutputDialog::on_showPoseRCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bPoseRToggle = toggle;
    UpdateAllDisplay();
}

void OutputDialog::on_showGlobalTCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bGlobalTToggle = toggle;
    UpdateAllDisplay();
}

void OutputDialog::on_showGlobalRCheckBox_toggled(bool toggle)
{
    m_debugCtrl.m_bGlobalRToggle = toggle;
    UpdateAllDisplay();
}

void OutputDialog::on_buttonBox_accepted()
{
    m_bShown = false;
}

void OutputDialog::on_buttonBox_rejected()
{
    m_bShown = false;
}

void OutputDialog::on_Clear_clicked()
{
    ClearString(m_nSelectStr);
}

void OutputDialog::on_Save_clicked()
{
    QString fileName = QFileDialog::getSaveFileName(this, tr("保存结果"), "/User");
    QFile   file(fileName);

    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) // 以文本文式写入
    {
        QTextStream out(&file);

        for (vector<QString*>::iterator it = m_savedStr[m_nSelectStr].begin(); it != m_savedStr[m_nSelectStr].end(); it++)
        {
            QString *p = *it;
            out << *p;
            out << endl;
        }

        file.close();
    }
}

void OutputDialog::ClearString(int index)
{
    if (index < 0 || index >= TQC_TOTAL_POSES_ARRAY_NUM)
        return;

    for (vector<QString*>::iterator it = m_savedStr[index].begin(); it != m_savedStr[index].end();)
    {
        QString *p = *it;
        it = m_savedStr[index].erase(it);
        delete p;
    }
}

void OutputDialog::SetSelectStr(int index)
{
    if (index < 0 || index >= TQC_TOTAL_POSES_ARRAY_NUM)
        return;

    m_nSelectStr = index;
}

void OutputDialog::SaveString(int nSelect, char *fmt, ...)
{
    char    buf[256];
    va_list arg_ptr;

    va_start(arg_ptr, fmt);
    memset(buf, 0, 256);
    vsprintf(buf, fmt, arg_ptr);
    va_end(arg_ptr);

    QString *p = new QString(tr(buf));
    m_savedStr[nSelect].push_back(p);
}

void OutputDialog::OutputString(int nView, char *fmt, ...)
{
    QTextBrowser *pView = GetView(nView);
    char         buf[256];
    va_list      arg_ptr;

    va_start(arg_ptr, fmt);
    memset(buf, 0, 256);
    vsprintf(buf, fmt, arg_ptr);
    va_end(arg_ptr);

    pView->append(tr(buf));
}