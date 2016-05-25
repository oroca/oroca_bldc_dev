QT += core
QT -= gui
QT += serialport


CONFIG += c++11

TARGET = test1
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    test_send_mavlink.cpp \
    serialportwriter.cpp \
    serialportreader.cpp

HEADERS += \
    serialportwriter.h \
    serialportreader.h
