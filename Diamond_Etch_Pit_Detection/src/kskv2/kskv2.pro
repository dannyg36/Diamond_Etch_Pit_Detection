QT       += widgets core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    MyLabel.cpp \
    main.cpp \
    mainwindow.cpp \
    waiting.cpp

HEADERS += \
    MyLabel.h \
    mainwindow.h \
    waiting.h

FORMS += \
    mainwindow.ui \
    waiting.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

INCLUDEPATH += /usr/local/include/opencv4
LIBS += -L//usr/local/lib\
-lopencv_core \
 -lopencv_highgui \
 -lopencv_imgproc \
  -lopencv_imgcodecs \
  -lopencv_videoio \
  -lopencv_imgproc \
  -lopencv_imgcodecs \
  -lopencv_video \
  -lopencv_calib3d \
  -lopencv_features2d \
  -lopencv_ml \
  -lopencv_objdetect \
