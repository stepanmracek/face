#include <QApplication>
#include <QString>

#include "facelib/morphable3dfacemodel.h"
#include "facelib/morphable3dfacemodelwidget.h"

int main(int argc, char *argv[])
{
    QString pca = "../../test/morph-pca.xml";
    QString pcaZcoord = "../../test/morph-pca-zcoord.xml";
    QString pcaTexture = "../../test/morph-pca-texture.xml";
    QString flags = "../../test/morph-flags";
    QString landmarksPath = "../../test/morph-landmarks.xml";

    Morphable3DFaceModel model(pcaZcoord, pcaTexture, pca, flags, landmarksPath, 200);

    QApplication app(argc, argv);
    Morphable3DFaceModelWidget widget;
    widget.setModel(&model);
    widget.setWindowTitle("GL Widget");
    widget.show();

    return app.exec();
}
