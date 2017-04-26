#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QtOpenGL/QGLWidget>

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/facedata/landmarks.h"
#include "faceExtras/faceExtras.h"

namespace Face {
namespace GUI {

/**
 * Class form painting 3D model of the face
 */
class FACEEXTRAS_EXPORTS GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    /**
     * Constructor
     */
    GLWidget(QWidget *parent = 0);
    /**
     * Destructor
     */
    ~GLWidget();
    /**
     * Occurs when widget is resized
     * @param width new width
     * @param height new height
     */
    void resizeGL(int width, int height);

    void addFace(const Face::FaceData::Mesh *f) { faces.push_back(f) ; updateGL(); }
    void addLandmarks(Face::FaceData::Landmarks *l) { this->landmarks.push_back(l); }
    void addCurve(std::vector<cv::Point3d> &c) { curves.push_back(c); }
    void deleteAll();
    void clearAll();

    const Face::FaceData::Mesh *getFace() { if (faces.empty()) return 0; else return faces[0]; }

protected:
    void init();
    void initializeGL();
    void paintGL();
    void refreshData();
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
    void timerEvent(QTimerEvent *e);

private:
    std::vector<const Face::FaceData::Mesh*> faces;
    std::vector<Face::FaceData::Landmarks*> landmarks;
    std::vector<std::vector<cv::Point3d> > curves;

    void  line(int i, int j);

    int   xnew, ynew, znew;                 /* soucasna pozice, ze ktere se pocitaji rotace a posuny */
    int   xold, yold, zold;                 /* minula pozice, ze ktere se pocitaji rotace a posuny */
    int   xx1, yy1, zz1;                    /* body, ve kterych se nachazi kurzor mysi */
    int   stav;                             /* stav tlacitek mysi */
};

}
}

#endif // GLWIDGET_H
