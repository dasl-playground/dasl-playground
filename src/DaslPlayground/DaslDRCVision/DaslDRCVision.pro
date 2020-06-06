QT -= gui

CONFIG += c++17 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src/DaslDRCVisionNode.cpp \
    src/main.cpp \
    src/subscriber.cpp

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    DaslPanMotionController.h \
    DaslRS232.h \
    daslpg.h \
    include/DaslDRCVisionNode.h \
    include/DaslLidarActionServerNode.h

DISTFILES += \
    CMakeLists.txt \
    package.xml

INCLUDEPATH += $$PWD/../../../../../../opt/ros/foxy/include
DEPENDPATH += $$PWD/../../../../../../opt/ros/foxy/include

unix:!macx: LIBS += -L$$PWD/../../DaslLibrary/ -ltest

INCLUDEPATH += $$PWD/../../DaslLibrary/include
DEPENDPATH += $$PWD/../../DaslLibrary/include


INCLUDEPATH += $$PWD/../../DaslPlayground/DaslDRCVision/include
DEPENDPATH += $$PWD/../../DaslPlayground/DaslDRCVision/include
