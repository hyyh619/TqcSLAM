#-------------------------------------------------
#
# Project created by QtCreator 2016-10-26T10:06:21
#
#-------------------------------------------------

QT += core gui opengl xml

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET   = TqcVisualOdemetry
TEMPLATE = app

SOURCES += main.cpp\
           TqcEncodeDecode.cpp \
           TqcCamera.cpp \
           TqcBundleAdjust.cpp \
           Tqc3dViewer.cpp \
           TqcScene.cpp \
           VoMainWindow.cpp \
           OutputDialog.cpp \
           TqcDebugControl.cpp \
    TqcThread.cpp \
    MatchDialog.cpp

HEADERS += \
           TqcEncodeDecode.h \
           TqcCamera.h \
           TqcBundleAdjust.h \
           Tqc3dViewer.h \
           TqcScene.h \
           VoMainWindow.h \
           OutputDialog.h \
           TqcDebugControl.h \
           TqcDefines.h \
           TqcTypes.h \
    TqcThread.h \
    MatchDialog.h

FORMS   += vomainwindow.ui \
    OutputDialog.ui \
    MatchDialog.ui

# include
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/release/ -lQGLViewer
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/debug/ -lQGLViewer
else:mac:INCLUDEPATH += /usr/local/include
else:unix: LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/ -lQGLViewer

# lib
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/release/ -lQGLViewer
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/debug/ -lQGLViewer
else:mac:LIBS += -L/usr/local/lib \
                 -lopencv_core \
                 -lopencv_videostab \
                 -lopencv_video \
                 -lopencv_videoio \
                 -lopencv_viz \
                 -lopencv_superres \
                 -lopencv_stitching \
                 -lopencv_shape \
                 -lopencv_photo \
                 -lopencv_objdetect \
                 -lopencv_calib3d \
                 -lopencv_ml \
                 -lopencv_highgui \
                 -lopencv_imgcodecs \
                 -lopencv_flann \
                 -lopencv_imgproc\
                 -lopencv_features2d
else:unix: LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/ -lQGLViewer

# QGLViewer
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/release/ -lQGLViewer
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/debug/ -lQGLViewer
else:mac: LIBS += -L/Users/yinghuang/Qt/Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/QGLViewer.framework/Versions/2/ /Users/yinghuang/Qt/Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/build/Debug/QGLViewer.framework/Versions/A/QGLViewer
else:unix: LIBS += -L$$PWD/../../../Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/ -lQGLViewer

# QGLViewer include folders
INCLUDEPATH += /Users/yinghuang/Qt/Examples/QGLView/libQGLViewer-2.6.4/QGLViewer/QGLViewer.framework/Headers
DEPENDPATH += /Users/yinghuang/Qt/Examples/QGLView/libQGLViewer-2.6.4/QGLViewer

# Add g2o lib and header
INCLUDEPATH += /Users/yinghuang/development/hy_code/SLAM/ORB_SLAM/ORB_SLAM2_for_osx/eigen-3.3-rc1 \
               /Users/yinghuang/development/hy_code/SLAM/g2o_ba_example/g2o-master/g2o/solvers/cholmod \
               /Users/yinghuang/development/hy_code/SLAM/g2o_ba_example/g2o-master \
               /Users/yinghuang/development/hy_code/SLAM/g2o_ba_example/g2o-master/build \
               /Users/yinghuang/development/hy_code/SLAM/g2o_ba_example/SuiteSparse/SuiteSparse_config \
               /Users/yinghuang/development/hy_code/SLAM/g2o_ba_example/SuiteSparse/CHOLMOD/Include
LIBS += -L/Users/yinghuang/development/hy_code/SLAM/g2o_ba_example/SuiteSparse/lib \
        -L/Users/yinghuang/development/hy_code/SLAM/g2o_ba_example/g2o-master/lib/Debug \
        -lg2o_core_d \
        -lg2o_types_slam3d_d \
        -lg2o_solver_csparse_d \
        -lg2o_stuff_d \
        -lg2o_csparse_extension_d \
        -lg2o_types_sba_d \
        -lcholmod

# Add ffmpeg
INCLUDEPATH += ／usr/local/include

LIBS += -L/usr/local/lib \
        -lavcodec \
        -lx264 \
        -lmp3lame \
        -lavformat \
        -lavdevice \
        -lavfilter \
        -lavutil \
        -lswscale
