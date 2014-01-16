#include <QApplication>
#include <QString>
#include <QInputDialog>

#include "kinect.h"
#include "facelib/glwidget.h"
#include "facelib/facefeaturesanotation.h"
#include "facelib/facealigner.h"
#include "facelib/surfaceprocessor.h"
#include "dlgscanface.h"
#include "frmkinectmain.h"
#include "kinectsensorplugin.h"

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

int mainMSV(int argc, char *argv[])
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

    KinectSensorPlugin plugin(p.haarFaceDetectPath, p.alignReferencePath);
    FaceClassifier classifier(p.classifierDirPath);
    FrmKinectMain frmMain(plugin, classifier, p.databasePath);
    frmMain.show();

    return app.exec();
}

int mainKinect(int argc, char *argv[])
{
    KinectSensorPlugin plugin("../../test/haar-face.xml", "../../test/meanForAlign.obj");

    if (!plugin.isKinectPluggedIn())
    {
        qDebug() << "Kinect not plugged in";
        return 0;
    }
    else
    {
        qDebug() << "Kinect detected";
    }

    //plugin.scanFace();
    //qDebug() << "Second shot";
    plugin.scan();
    Mesh mesh = plugin.mesh();

    //mesh->colors.clear();
    SurfaceProcessor::smooth(mesh, 0.7, 5);

    QApplication app(argc, argv);
    GLWidget widget;
    widget.addFace(&mesh);
    widget.show();
    return app.exec();
}

int main(int argc, char *argv[])
{
    mainMSV(argc, argv);
}
