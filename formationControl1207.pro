QT += core
QT -= gui

CONFIG += c++11

TARGET = formationControl1207
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    sensor/MPU6050.cpp \
    sensor/I2Cdev.cpp \
    controller/kinematiccontroller.cpp \
    controller/lineformationcontroller.cpp \
    driver/motor.cpp \
    driver/servo.cpp \
    util/cameraposition.cpp \
    util/dataLib.cpp \
    util/differentialdrive.cpp \
    util/kalman.cpp \
    util/undistortion.cpp \
    sensor/rotaryencoder.cpp \
    sensor/demo_dmp.cpp \
    carstatus.cpp \
    util/log.cpp

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

HEADERS += \
    sensor/MPU6050_6Axis_MotionApps20.h \
    sensor/MPU6050.h \
    sensor/I2Cdev.h \
    controller/kinematiccontroller.h \
    controller/lineformationcontroller.h \
    driver/motor.h \
    driver/servo.h \
    util/cameraposition.h \
    util/dataLib.h \
    util/differentialdrive.h \
    util/kalman.h \
    util/undistortion.h \
    sensor/rotaryencoder.h \
    sensor/demo_dmp.h \
    sensor/helper_3dmath.h \
    globalsettings.h \
    carstatus.h \
    util/log.h

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lraspicam_cv -lraspicam -lwiringPi -lopencv_calib3d -lARToolKitPlus -lopencv_imgcodecs -lopencv_core -lopencv_highgui -lopencv_imgproc



INCLUDEPATH += $$PWD/../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lARToolKitPlus

INCLUDEPATH += $$PWD/../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../usr/local/include