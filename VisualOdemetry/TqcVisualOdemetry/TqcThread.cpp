#include <QDebug>
#include "TqcThread.h"

CThread::CThread()
{
    m_bStop = false;
}

void CThread::run()
{
    while (!m_bStop)
    {
        PrintMessage();
    }

    m_bStop = false;
}

void CThread::stop()
{
    m_bStop = true;
}

void CThread::SetMessage(QString message)
{
    m_msgStr = message;
}

void CThread::PrintMessage()
{
    qDebug() << m_msgStr;
    sleep(1);
}