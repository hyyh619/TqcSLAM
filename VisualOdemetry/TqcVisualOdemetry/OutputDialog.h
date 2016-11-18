#ifndef OUTPUTDIALOG_H
#define OUTPUTDIALOG_H

#include <QDialog>
#include <QTextBrowser>
#include <QString>
#include <vector>
#include "TqcDefines.h"
#include "TqcTypes.h"
#include "TqcDebugControl.h"

using namespace std;

namespace Ui
{
class OutputDialog;
}

class OutputDialog : public QDialog
{
    Q_OBJECT

public:
    explicit OutputDialog(QWidget *parent = 0);
    ~OutputDialog();

    void OutputString(int nView, char *fmt, ...);
    void SaveString(int nSelect, char *fmt, ...);
    void ClearString(int index);
    void SetSelectStr(int index);
    void UpdateDisplay(int nView, int nSelect);
    void UpdateAllDisplay();
    void LoadInformFromPose(int nLoad, CPose *p);
    QTextBrowser* GetView(int nView);
    void UpdateChannelName(int nLoad, QString name);

public slots:
    void on_buttonBox_accepted();
    void on_buttonBox_rejected();
    void on_Save_clicked();
    void on_Clear_clicked();
    void on_showMatrixCheckBox_toggled(bool toggle);
    void on_showPoseTCheckBox_toggled(bool toggle);
    void on_showPoseRCheckBox_toggled(bool toggle);
    void on_showGlobalTCheckBox_toggled(bool toggle);
    void on_showGlobalRCheckBox_toggled(bool toggle);
    void on_channelComboBox_currentIndexChanged(int index);
    void on_channelComboBox2_currentIndexChanged(int index);

private:
    Ui::OutputDialog *ui;

public:
    CDebugControl m_debugCtrl;
    bool          m_bShown;
    StringArray   m_savedStr[TQC_TOTAL_POSES_ARRAY_NUM];
    int           m_nSelectStr;
    int           m_nSelectStr2;
};
#endif // OUTPUTDIALOG_H