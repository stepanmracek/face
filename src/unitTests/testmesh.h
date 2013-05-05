#ifndef TESTMESH_H
#define TESTMESH_H

#include <QApplication>
#include <QDir>
#include <QFileInfoList>

#include "facelib/glwidget.h"
#include "facelib/mesh.h"
#include "facelib/landmarks.h"
#include "facelib/surfaceprocessor.h"

class TestMesh
{
public:
    static void testXYZLodaderOBJWriter(const QString &srcDirPath, const QString &dstDirPath)
    {
        QDir srcDir(srcDirPath, "*.xyz");
        QFileInfoList srcEntries = srcDir.entryInfoList();
        foreach (const QFileInfo &srcFileInfo, srcEntries)
        {
            Mesh srcMesh = Mesh::fromXYZ(srcFileInfo.absoluteFilePath(), true);
            QString dstFilePath = dstDirPath + QDir::separator() + srcFileInfo.baseName() + ".obj";
            srcMesh.writeOBJ(dstFilePath, '.');
        }
    }

    static int testBinReadWrite(int argc, char *argv[], const QString &dir)
    {
        Mesh mesh = Mesh::fromXYZ(dir + "xyz/02463d652.abs.xyz", false);

        mesh.writeBIN("face.bin");
        Mesh mesh2 = Mesh::fromBIN("face.bin");

        QApplication app(argc, argv);
        GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh2);
        widget.show();
        return app.exec();
    }
};

#endif // TESTMESH_H

