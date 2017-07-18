#include "widget.h"

#define SCALE  * 0.1

Widget::Widget(QWidget *parent, const QGLWidget *shareWidget, Qt::WindowFlags flags) : QGLViewer(parent, shareWidget, flags)
{
    id = 0;
}

void Widget::animate()
{
    id++;
    if(id == simulation.poses.size())
        id = 0;
}

void Widget::draw()
{
    glPointSize(5);
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_POINTS);
    if(!simulation.points.empty()) {
        for(size_t i = 0; i < simulation.points.size(); ++i) {
            glVertex3d(simulation.points[i][0] SCALE, simulation.points[i][1] SCALE, simulation.points[i][2] SCALE);
        }
    }
    glEnd();
    drawCam();
}

void Widget::init()
{
    gluLookAt (0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    Eigen::Matrix2d featVar;
    featVar(0, 0) = 0.001;
    featVar(1, 1) = 0.0005;
    featVar(0, 1) = featVar(1, 0) = 0.000001;
    simulation.run(featVar);
}

void Widget::drawCam()
{
    if(simulation.poses.empty())
        return;
    glLineWidth(2);
    Eigen::Vector3d top(0, 0, 0), A(5, 5 , 2), B(5, -5, 2), C(-5, -5, 2), D(-5, 5, 2);
    top = simulation.poses[id].pose * top SCALE;
    A = simulation.poses[id].pose * A SCALE;
    B = simulation.poses[id].pose * B SCALE;
    C = simulation.poses[id].pose * C SCALE;
    D = simulation.poses[id].pose * D SCALE;

    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
    glNormal3f(0, 0, 1);
    glVertex3d(A(0), A(1), A(2));
    glVertex3d(B(0), B(1), B(2));

    glVertex3d(B(0), B(1), B(2));
    glVertex3d(C(0), C(1), C(2));

    glVertex3d(C(0), C(1), C(2));
    glVertex3d(D(0), D(1), D(2));

    glVertex3d(D(0), D(1), D(2));
    glVertex3d(A(0), A(1), A(2));

    glVertex3d(top(0), top(1), top(2));
    glVertex3d(A(0), A(1), A(2));

    glVertex3d(top(0), top(1), top(2));
    glVertex3d(B(0), B(1), B(2));

    glVertex3d(top(0), top(1), top(2));
    glVertex3d(C(0), C(1), C(2));

    glVertex3d(top(0), top(1), top(2));
    glVertex3d(D(0), D(1), D(2));

    glEnd();

    glPointSize(2);
    glColor3f(1.0, 0, 0);
    glBegin(GL_POINTS);
    for(size_t i = 0; i < simulation.poses[id].projectPoints.size(); ++i) {
            glVertex3d(simulation.poses[id].projectPoints[i][0] SCALE, simulation.poses[id].projectPoints[i][1] SCALE, simulation.poses[id].projectPoints[i][2] SCALE);
    }
    glEnd();


    Eigen::Vector3d top1(0, 0, 0), A1(5, 5 , 2), B1(5, -5, 2), C1(-5, -5, 2), D1(-5, 5, 2);
    top1 = simulation.poses[id].noisePose * top1 SCALE;
    A1 = simulation.poses[id].noisePose * A1 SCALE;
    B1 = simulation.poses[id].noisePose * B1 SCALE;
    C1 = simulation.poses[id].noisePose * C1 SCALE;
    D1 = simulation.poses[id].noisePose * D1 SCALE;
    glLineWidth(1);
    glColor3f(0.0, 1.0, 1.0);
    glBegin(GL_LINES);
    glNormal3f(0, 0, 1);
    glVertex3d(A1(0), A1(1), A1(2));
    glVertex3d(B1(0), B1(1), B1(2));

    glVertex3d(B1(0), B1(1), B1(2));
    glVertex3d(C1(0), C1(1), C1(2));

    glVertex3d(C1(0), C1(1), C1(2));
    glVertex3d(D1(0), D1(1), D1(2));

    glVertex3d(D1(0), D1(1), D1(2));
    glVertex3d(A1(0), A1(1), A1(2));

    glVertex3d(top1(0), top1(1), top1(2));
    glVertex3d(A1(0), A1(1), A1(2));

    glVertex3d(top1(0), top1(1), top1(2));
    glVertex3d(B1(0), B1(1), B1(2));

    glVertex3d(top1(0), top1(1), top1(2));
    glVertex3d(C1(0), C1(1), C1(2));

    glVertex3d(top1(0), top1(1), top1(2));
    glVertex3d(D1(0), D1(1), D1(2));

    glEnd();

}

