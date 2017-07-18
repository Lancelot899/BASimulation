#ifndef WIDGET_H
#define WIDGET_H

#include <QGLViewer/qglviewer.h>

#include "simulation/simulation.h"

class Widget : public QGLViewer
{
public:
    Widget(QWidget* parent=0, const QGLWidget* shareWidget=0, Qt::WindowFlags flags=0);

    virtual void animate();
    virtual void draw();
    virtual void init();

private:
    void drawCam();

private:
    Simulation simulation;
    size_t id;

};

#endif // WIDGET_H
