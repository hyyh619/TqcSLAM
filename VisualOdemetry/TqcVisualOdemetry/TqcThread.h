#ifndef TQCTHREAD_H
#define TQCTHREAD_H

#include <QThread>
#include <iostream>

class CThread : public QThread
{
    Q_OBJECT
public:
    CThread();
    void SetMessage(QString message);
    void stop();

protected:
    void run();
    void PrintMessage();

protected:
    QString       m_msgStr;
    volatile bool m_bStop;
};
#endif // TQCTHREAD_H