#include "glwidget.h"

#include <opencv/cv.h>
#include <GL/freeglut.h>
#include <GL/freeglut_std.h>
#include <GL/freeglut_ext.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

GLWidget::GLWidget(QWidget* parent) : QGLWidget(parent)
{
    init();
}

void GLWidget::init()
{
    //refreshData();
    xnew = ynew = znew = xold = yold = zold = xx1 = yy1 = zz1 = stav = 0;
    //startTimer(100);
}

GLWidget::~GLWidget()
{
}

void GLWidget::initializeGL()
{
    glClearColor(0.0f,0.0f,0.0f,0.0f);

    //glHint( GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST );
    //glEnable( GL_TEXTURE_2D );
    //glEnable( GL_CULL_FACE );
    glEnable( GL_DEPTH_TEST );

    //glEnable(GL_COLOR_MATERIAL);
    //glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
}

void GLWidget::refreshData()
{
    /*if (!Kinect::getRGB(rgb))
    {
        std::cout << "Kinect rgb error" << std::endl;
        exit(1);
    }
    if (!Kinect::getDepth(depth, 20, NULL, 200, 1500))
    {
        std::cout << "Kinect depth error" << std::endl;
        exit(1);
    }

    face = Kinect::createMesh(depth, rgb);
    SurfaceProcessor::centralize(face);
    updateGL();*/
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    //glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLoadIdentity();

    glTranslatef(0.0, 0.0, -400.0);
    glTranslatef(0, 0, znew);
    glRotatef(xnew, 1, 0, 0);
    glRotatef(ynew, 0, 1, 0);

    //if (!face) return;

    //float color[] = {0.75f,0.75f,0.75f};
    float color[] = {214.0/255.0, 180.0/255.0, 155.0/255.0};
    glColor3fv(color);
    foreach(Mesh *face, faces)
    {
        if (face->triangles.count() == 0)
        {
            glDisable(GL_LIGHTING);
            glDisable(GL_LIGHT0);

            glBegin(GL_POINTS);
            int n = face->points.size();
            for (int i = 0; i < n; i++)
            {
                cv::Point3d &p = face->points[i];
                Color &c = face->colors[i];
                glColor3b(c[2]/2,c[1]/2,c[0]/2);
                glVertex3f(p.x/10, p.y/10, p.z/10);

            }
            glEnd();
        }
        else
        {
            bool texture = face->colors.size() == face->points.size();
            if (!texture)
            {
                glEnable(GL_LIGHTING);
                glEnable(GL_LIGHT0);
                glMaterialfv(GL_FRONT, GL_DIFFUSE, color);
            }

            glPointSize(1);
            glBegin(GL_TRIANGLES);
            int c = face->triangles.count();
            for (int i = 0; i < c; i++)
            {
                int p1 = face->triangles[i][0];
                int p2 = face->triangles[i][1];
                int p3 = face->triangles[i][2];

                // U = p2 - p1
                float ux = face->points[p2].x - face->points[p1].x;
                float uy = face->points[p2].y - face->points[p1].y;
                float uz = face->points[p2].z - face->points[p1].z;

                // V = p3 - p1
                float vx = face->points[p3].x - face->points[p1].x;
                float vy = face->points[p3].y - face->points[p1].y;
                float vz = face->points[p3].z - face->points[p1].z;

                // n = cross(U,V)
                float nx = uy*vz - uz*vy;
                float ny = uz*vx - ux*vz;
                float nz = ux*vy - uy*vx;

                // normalize
                float size = 1.0f/sqrt(nx*nx + ny*ny + nz*nz);
                nx = nx * size;
                ny = ny * size;
                nz = nz * size;

                // flat shading
                glNormal3f(nx, ny, nz);

                if (texture)
                {
                    glColor3b(face->colors[p1][2]/2,face->colors[p1][1]/2,face->colors[p1][0]/2);
                }
                glVertex3f(face->points[p1].x, face->points[p1].y, face->points[p1].z);

                if (texture)
                {
                    glColor3b(face->colors[p2][2]/2,face->colors[p2][1]/2,face->colors[p2][0]/2);
                }
                glVertex3f(face->points[p2].x, face->points[p2].y, face->points[p2].z);

                if (texture)
                {
                    glColor3b(face->colors[p3][2]/2,face->colors[p3][1]/2,face->colors[p3][0]/2);
                }
                glVertex3f(face->points[p3].x, face->points[p3].y, face->points[p3].z);
            }

            glEnd();
        }
    }

    foreach(Landmarks *l, landmarks)
    {
        glDisable(GL_LIGHTING);
        glDisable(GL_LIGHT0);

        glLineWidth(5);
        glColor3f(1,1,0);

        glBegin(GL_LINES);
        foreach (const cv::Point3d &p, l->points)
        {
            glVertex3f(p.x, p.y, p.z);
            glVertex3f(p.x, p.y, p.z+3);
        }
        glEnd();
    }

    if (curves.count() > 0)
    {
        glDisable(GL_LIGHTING);
        glDisable(GL_LIGHT0);

        glLineWidth(3);
        glColor3f(1,1,0);

        foreach (const QVector<cv::Point3d> &curve, curves)
        {
            glBegin(GL_LINE_STRIP);
            int n = curve.count();
            for (int i = 0; i <= n; i++)
            {
                cv::Point3d &p = curve[i%n];
                if (p.x == p.x && p.y == p.y && p.z == p.z)
                {
                    glVertex3d(p.x, p.y, p.z+3);
                }
            }
            glEnd();
        }
    }
}


void GLWidget::resizeGL(int width, int height)
{
    //proces resize keep good aspect ratio for 3D scene
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    int side = qMax(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45.0, 1.0, 0.01, 1000.0);
    glMatrixMode(GL_MODELVIEW);
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
  int x = event->y();
  int y = event->x();

  if (event->button() == Qt::LeftButton) {             /* leve tlacitko aktivuje rotaci */
      stav = 1;                                 /* nastaveni pro funkci motion */
      xx1 = x;                                  /* zapamatovat pozici kurzoru mysi */
      yy1 = y;
  }
  if (event->button() == Qt::RightButton) {            /* prave tlacitko aktivuje posun */
      stav = 2;                                 /* nastaveni pro funkci motion */
      zz1 = y;                                  /* zapamatovat pozici kurzoru mysi */

  }
  updateGL();
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    //int x = event->y();
    //int y = event->x();

    if (event->button() == Qt::LeftButton) {             /* leve tlacitko aktivuje rotaci */
        stav = 0;                                 /* normalni stav */
        xold = xnew;                              /* zapamatovat novy pocatek */
        yold = ynew;
    }
    if (event->button() == Qt::RightButton) {            /* prave tlacitko aktivuje posun */
        // = 0;
        zold = znew;                              /* zapamatovat novy pocatek */
    }
    updateGL();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    //std::cout << "move " << stav << " ";
    int x = event->y();
    int y = event->x();
    //std::cout << x << " " << y << std::endl;
    if (stav == 1) {                              /* stav presunu */
        xnew = xold+x-xx1;                          /* vypocitat novou pozici */
        ynew = yold+y-yy1;
        updateGL();
    }
    if (stav == 2)
    {                              // stav rotace
        znew = zold+y-zz1;         // vypocitat novou pozici
        updateGL();
    }
}
