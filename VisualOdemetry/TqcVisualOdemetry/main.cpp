#include "VoMainWindow.h"
#include <QApplication>

/*
 * Todo:
 * 1. 为trackView添加线条连接两个Vertex。                           ---暂时不需要
 * 2. H.264设置码率，控制图像质量。
 * 3. 特征点匹配后，需要过滤一些不合适的情况。
 */

/*
 * 已完成:
 * 1.  增加步进按钮                                                 ---完成
 * 2.  增加选择顶点功能，点选或者通过数字选择后，在DebugOutput中输出起R和t  ---完成
 * 3.  增加控制输出选项，可以输出更多信息                               ---完成
 * 4.  trackView绘制顶点时，需要加上序号，方便查询。                     ---完成
 * 5.  能够输出运行结果，重新加载和绘制。                                ---完成
 * 6.  增加按帧数跳转                                                ---完成
 * 7.  绘制三角形大小不随视角拉近而变大                                  ---完成
 * 8.  OutputDialog增加双视窗输出，方便结果比对                         ---完成
 * 9.  trackView和DebugOutput控件之间可以拖动splitter实现窗口高度控制。   ---完成
 * 10. 把SLAM计算和界面，视频等分开到不同的线程。                         ---完成SLAM计算线程，视频未采用多线程。
 * 11. 每帧得到的顶点绘制时需要计算与Camera的距离来确定其绘制大小。          ---完成
 * 12. 限制顶点三角绘制大小
 * 13. 优化了暂停和单步跟踪功能
 * 14. OutputDialog中的channel选择提供文件名
 * 15. 提供帧间对比的结果框，例如两帧图像之间的数据对比，特征点的匹配情况。
 */

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    VOMainWindow w;

    w.show();

    return a.exec();
}