#-------------------------------------------------
#
# Project created by QtCreator 2018-07-09T01:59:42
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = uitest
TEMPLATE = app

CONFIG += c++11

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    carstatus.cpp \
    differentialdrive.cpp \
    kalman.cpp \
    kinematiccontroller.cpp \
    line_formation_control.cpp \
    log.cpp \
    motor.cpp \
    pid.cpp \
    udp_client.cpp \
    mpu/demo_dmp.cpp \
    mpu/I2Cdev.cpp \
    mpu/MPU6050.cpp \
    rotaryencoder.cpp \
    utils.cpp

HEADERS  += mainwindow.h \
    carstatus.h \
    differentialdrive.h \
    globalsettings.h \
    kalman.h \
    kinematiccontroller.h \
    line_formation_control.h \
    log.h \
    motor.h \
    pid.h \
    udp_client.h \
    utils.h \
    mpu/demo_dmp.h \
    mpu/helper_3dmath.h \
    mpu/I2Cdev.h \
    mpu/MPU6050.h \
    mpu/MPU6050_6Axis_MotionApps20.h \
    rotaryencoder.h

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lwiringPi -lopencv_calib3d -lARToolKitPlus -lopencv_imgcodecs -lopencv_core -lopencv_highgui -lopencv_imgproc



INCLUDEPATH += $$PWD/../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../usr/local/include

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lARToolKitPlus

INCLUDEPATH += $$PWD/../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../usr/local/include

DISTFILES +=

FORMS    += mainwindow.ui
