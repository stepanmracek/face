#include <QApplication>
#include <QString>

#include "kinect.h"
#include "facelib/glwidget.h"
#include "facelib/morphable3dfacemodel.h"
#include "facelib/facefeaturesanotation.h"
#include "facelib/facealigner.h"

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
    Mesh inputMesh = Mesh::fromBIN("../../test/kinect-face.bin", false);
    Landmarks landmarks("../../test/kinect-face.xml");

    QString pca = "../../test/morph-pca.xml";
    QString pcaZcoord = "../../test/morph-pca-zcoord.xml";
    QString pcaTexture = "../../test/morph-pca-texture.xml";
    QString flags = "../../test/morph-flags";
    QString landmarksPath = "../../test/morph-landmarks.xml";
    Morphable3DFaceModel model(pcaZcoord, pcaTexture, pca, flags, landmarksPath, 200);

    Procrustes3DResult procrustesResult = model.align(inputMesh, landmarks, 10);
    model.morphModel(inputMesh);
    Procrustes3D::applyInversedProcrustesResult(model.mesh.points, procrustesResult);
    Procrustes3D::applyInversedProcrustesResult(inputMesh.points, procrustesResult);
    model.mesh.recalculateMinMax();
    inputMesh.recalculateMinMax();

    Mesh meanForAlign = Mesh::fromOBJ("../../test/meanForAlign.obj");
    FaceAligner aligner(meanForAlign);
    aligner.icpAlign(model.mesh, 10);

    QApplication app(argc, argv);
    GLWidget widget;
    widget.addFace(&model.mesh);
    widget.addFace(&meanForAlign);
    widget.show();

    return app.exec();
}

int main(int argc, char *argv[])
{
    return align(argc, argv);
    //return scan(argc, argv, "../../test/kinect-face.bin", "../../test/kinect-face.xml");
}
