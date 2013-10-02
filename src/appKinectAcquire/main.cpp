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
    Mesh *m = Kinect::scanFace(10, "../../test/haar-face.xml");
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
    Mesh *m = Kinect::scanFace(10, "../../test/haar-face.xml");
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
    Mesh *inputMesh = Kinect::scanFace(10, "../../test/haar-face.xml"); // Mesh::fromBIN("../../test/kinect-face.bin", false);
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

struct Arguments
{
    QString classifierDirPath;
    QString databasePath;
    QString alignReferencePath;
    QString haarFaceDetectPath;
};

void printHelp(const QString appName)
{
    QTextStream out(stdout);
    out << "Usage:\n";
    out << appName << " -db <database> -c <classifier> -a <align model> -h <haar face detect>\n";
    out << "  <database>         - path to the directory with initial state of the database\n";
    out << "  <classifier>       - path to the directory that contains serialized face classifier\n";
    out << "  <align model>      - path to the OBJ model used for proper face alignmnet\n";
    out << "  <haar face detect> - path to the haar face detector\n";
}

QString getArgumentValue(const QString &param, const QStringList &args, bool *ok)
{
    *ok = true;
    int index = args.indexOf(QString(param));
    if (index == -1 || index + 1 == args.count()) { *ok = false; return QString(); }
    return args[index+1];
}

Arguments parseArguments(const QStringList &args, bool *ok)
{
    Arguments p;

    p.databasePath = getArgumentValue("-db", args, ok);
    if (!*ok) return p;
    p.classifierDirPath = getArgumentValue("-c", args, ok);
    if (!*ok) return p;
    p.alignReferencePath = getArgumentValue("-a", args, ok);
    if (!*ok) return p;
    p.haarFaceDetectPath = getArgumentValue("-h", args, ok);

    return p;
}

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QStringList args = app.arguments();
    bool ok;
    Arguments p = parseArguments(args, &ok);
    if (!ok)
    {
        printHelp(args[0]);
        exit(0);
    }

    FaceClassifier classifier(p.classifierDirPath);
    FrmKinectMain frmMain(p.databasePath, classifier, p.alignReferencePath, p.haarFaceDetectPath);
    frmMain.show();

    return app.exec();
}
