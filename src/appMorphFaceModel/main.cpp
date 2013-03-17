#include <QApplication>
#include <QString>

#include "facelib/morphable3dfacemodel.h"
#include "facelib/morphable3dfacemodelwidget.h"

int main(int argc, char *argv[])
{
    QString pca = "../../test/align-pca.xml";
    QString flags = "../../test/align-flags";
    QString landmarksPath = "../../test/align-landmarks.xml";

    Morphable3DFaceModel model(pca, flags, landmarksPath, 200);

    QApplication app(argc, argv);
    Morphable3DFaceModelWidget widget;
    widget.setModel(&model);
    widget.setWindowTitle("GL Widget");
    widget.show();

    return app.exec();
}
