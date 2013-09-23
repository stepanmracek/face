#include <QApplication>
#include <QString>
#include <QInputDialog>

#include "kinect.h"
#include "facelib/glwidget.h"
#include "facelib/morphable3dfacemodel.h"
#include "facelib/facefeaturesanotation.h"
#include "facelib/facealigner.h"
#include "facelib/surfaceprocessor.h"
#include "dlgscanface.h"

#include "frmkinectmain.h"

int scan(int argc, char *argv[])
{
    Mesh *m = Kinect::scanFace(10);
    Mesh mean = Mesh::fromOBJ("../../test/meanForAlign.obj");
    FaceAligner aligner(mean);
    aligner.icpAlign(*m, 10);

    MapConverter c;
    cv::imshow("scan", SurfaceProcessor::depthmap(*m, c, cv::Point2d(-75,-75), cv::Point2d(75,75), 2, Texture_I).toMatrix());

    QApplication app(argc, argv);

    GLWidget widget;
    //QString scanName = QInputDialog::getText(&widget, "Scan name", "Scan name:", QLineEdit::Normal, "");
    //widget.setWindowTitle(scanName);
    //m.writeBIN("../../test/kinect/" + scanName + ".bin");

    widget.addFace(m);
    widget.show();

    return app.exec();
}

int scan(int argc, char *argv[], const QString &outputPath, const QString &lmPath)
{
    Mesh *m = Kinect::scanFace(10);
    bool success;
    Landmarks lm = FaceFeaturesAnotation::anotate(*m, success);
    if (!success)
    {
        return 0;
    }

    QApplication app(argc, argv);
    GLWidget widget;
    widget.addFace(m);
    widget.addLandmarks(&lm);
    widget.show();

    m->writeBIN(outputPath);
    lm.serialize(lmPath);

    return app.exec();
}

int align(int argc, char *argv[])
{
    Mesh *inputMesh = Kinect::scanFace(10); // Mesh::fromBIN("../../test/kinect-face.bin", false);
    //Landmarks landmarks("../../test/kinect-face.xml");

    QString pca = "../../test/morph-pca.xml";
    QString pcaZcoord = "../../test/morph-pca-zcoord.xml";
    QString pcaTexture = "../../test/morph-pca-texture.xml";
    QString flags = "../../test/morph-flags";
    QString landmarksPath = "../../test/morph-landmarks.xml";
    Morphable3DFaceModel model(pcaZcoord, pcaTexture, pca, flags, landmarksPath, 200);

    Mesh morphedMesh = model.morph(*inputMesh, 10); //landmarks, 10);

    QApplication app(argc, argv);

    GLWidget widget;
    QString scanName = QInputDialog::getText(&widget, "Scan name", "Scan name:", QLineEdit::Normal, "");
    widget.setWindowTitle(scanName);
    morphedMesh.writeBIN("../../test/kinect/" + scanName + ".bin");

    //morphedMesh.translate(cv::Point3d(-50,0,0));
    //morphedMesh.colors.clear();
    widget.addFace(&morphedMesh);

    //inputMesh.translate(cv::Point3d(50,0,0));
    //widget.addFace(&inputMesh);
    widget.show();   

    return app.exec();
}

int main(int argc, char *argv[])
{
    FaceClassifier classifier("../../test/kinect/classifiers/");

    QApplication app(argc, argv);
    FrmKinectMain frmMain("../../test/kinect", classifier, "../../test/meanForAlign.obj");
    frmMain.show();

    return app.exec();
}
