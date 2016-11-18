#include "MatchDialog.h"
#include "ui_MatchDialog.h"

MatchDialog::MatchDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::MatchDialog)
{
    ui->setupUi(this);
}

MatchDialog::~MatchDialog()
{
    delete ui;
}

void MatchDialog::DrawMatch(Mat match, Mat goodMatch)
{
    QImage image   = QImage(match.data, match.cols, match.rows, QImage::Format_RGB888).rgbSwapped();
    QImage goodImg = QImage(goodMatch.data, goodMatch.cols, goodMatch.rows, QImage::Format_RGB888).rgbSwapped();
    int    imageW  = image.width();
    int    imageH  = image.height();
    int    outputW = ui->graphicsView->width();
    int    outputH = ui->graphicsView->height();
    float  fScaleH = (float)outputW / (float)imageW;
    float  fScaleW = (float)outputH / (float)imageH;

    if (fScaleH > fScaleW)
    {
        outputH = (int)(fScaleW * (float)imageH);
    }
    else
    {
        outputW = (int)(fScaleH * (float)imageW);
    }

    image = image.scaled(outputW, outputH);
    goodImg = goodImg.scaled(outputW, outputH);

    ui->graphicsView->setPixmap(QPixmap::fromImage(image));
    ui->graphicsView2->setPixmap(QPixmap::fromImage(goodImg));
}
