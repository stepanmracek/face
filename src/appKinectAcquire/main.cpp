#include <QApplication>
#include <QString>

#include "kinect.h"
#include "facelib/glwidget.h"
#include "facelib/morphable3dfacemodel.h"
#include "facelib/facefeaturesanotation.h"

int scan(int argc, char *argv[], const QString &outputPath, const QString &lmPath)
{
    Mesh m = Kinect::scanFace(10);
    bool success;
    Landmarks lm = FaceFeaturesAnotation::anotate(m, success);
    if (!success)
    {
        return 0;
    }

    QApplication app(argc, argv);
    GLWidget widget;
    widget.addFace(&m);
    widget.addLandmarks(&lm);
    widget.show();

    m.writeBIN(outputPath);
    lm.serialize(lmPath);

    return app.exec();
}

int align(int argc, char *argv[])
{
    Mesh m = Mesh::fromBIN("../../test/kinect-face.bin", false);
    Landmarks landmarks("../../test/kinect-face.xml");

    QString pca = "../../test/morph-pca.xml";
    QString pcaZcoord = "../../test/morph-pca-zcoord.xml";
    QString pcaTexture = "../../test/morph-pca-texture.xml";
    QString flags = "../../test/morph-flags";
    QString landmarksPath = "../../test/morph-landmarks.xml";
    Morphable3DFaceModel model(pcaZcoord, pcaTexture, pca, flags, landmarksPath, 200);

    Procrustes3DResult procrustesResult = model.align(m, landmarks, 10);
    model.morphModel(m);

    QApplication app(argc, argv);
    GLWidget widget;

    //Procrustes3D::applyInversedProcrustesResult(model.mesh.points, procrustesResult);
    //Procrustes3D::applyInversedProcrustesResult(m.points, procrustesResult);
    model.mesh.recalculateMinMax();
    m.recalculateMinMax();

    model.mesh.translate(cv::Point3d(75,0,0));
    m.translate(cv::Point3d(-75,0,0));

    widget.addFace(&m);
    widget.addFace(&model.mesh);
    //widget.addLandmarks(&landmarks);
    //widget.addLandmarks(&model.landmarks);
    widget.show();

    return app.exec();
}

int main(int argc, char *argv[])
{
    return align(argc, argv);
    //return scan(argc, argv, "../../test/kinect-face.bin", "../../test/kinect-face.xml");
}
