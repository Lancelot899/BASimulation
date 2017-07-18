TEMPLATE = app
QT += widgets gui opengl
QT += core
QT += xml

QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += ./thirdparty

SOURCES += main.cpp \
    simulation/simulation.cpp \
    widget.cpp

HEADERS += \
    simulation/simulation.h \
    widget.h

LIBS += -lQGLViewer -lGLU -lg2o_stuff -lg2o_cli -lg2o_core
