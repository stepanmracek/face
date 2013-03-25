#include <QApplication>
#include <QString>

#include "kinect.h"
#include "facelib/glwidget.h"
#include "facelib/morphable3dfacemodel.h"
#include "facelib/facefeaturesanotation.h"

int scan(int argc, char *argv[], const QString &outputPath)
{
    Mesh m = Kinect::scanFace();

    QApplication app(argc, argv);
    GLWidget widget;
    widget.addFace(&m);
    widget.show();

    m.writeOBJ(outputPath, ',');

    return app.exec();
}

int main(int argc, char *argv[])
{
    //scan(argc, argv, "face.obj");

    Mesh m = Mesh::fromOBJ("../../test/obj/stepan.obj"); // Kinect::scanFace();

    QString pca = "../../test/align-pca.xml";
    QString flags = "../../test/align-flags";
    QString landmarksPath = "../../test/align-landmarks.xml";
    Morphable3DFaceModel model(pca, flags, landmarksPath, 200);

    Landmarks landmarks = FaceFeaturesAnotation::anotate(m, 8);

    Procrustes3DResult procrustesResult = model.align(m, landmarks, 10);
    model.morphModel(m);

    QApplication app(argc, argv);
    GLWidget widget;


    //m.move(cv::Point3d(0,0,-50));
    Procrustes3D::applyInversedProcrustesResult(model.mesh.points, procrustesResult);
    Procrustes3D::applyInversedProcrustesResult(m.points, procrustesResult);
    model.mesh.recalculateMinMax();
    m.recalculateMinMax();
    //model.mesh.move(cv::Point3d(0,0,50));

    widget.addFace(&m);
    widget.addFace(&model.mesh);
    //widget.addLandmarks(&landmarks);
    //widget.addLandmarks(&model.landmarks);
    widget.show();

    return app.exec();
}
