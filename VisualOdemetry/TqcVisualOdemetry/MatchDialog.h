#ifndef MATCHDIALOG_H
#define MATCHDIALOG_H

#include <QDialog>
#include "TqcTypes.h"

using namespace cv;

namespace Ui
{
class MatchDialog;
}

class MatchDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MatchDialog(QWidget *parent = 0);
    ~MatchDialog();
    void DrawMatch(Mat match, Mat goodMatch);

private:
    Mat             m_drawMatchMat;
    Mat             m_drawGoodMatchMat;
    Ui::MatchDialog *ui;
};
#endif // MATCHDIALOG_H
