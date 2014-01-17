#ifndef TESTMESH_H
#define TESTMESH_H

#include <QApplication>
#include <QDir>
#include <QFileInfoList>

#include "gui/glwidget.h"
#include "facedata/mesh.h"
#include "facedata/landmarks.h"
#include "facedata/surfaceprocessor.h"

class TestMesh
{
public:

    static void testReadWriteCharArray(const QString &frgcPath)
    {
        QString path = frgcPath + "zbin-aligned/02463d652.binz";
        Face::FaceData::Mesh m1 = Face::FaceData::Mesh::fromBINZ(path);
        int n;
        char *data = m1.toCharArray(&n);
        Face::FaceData::Mesh m2 = Face::FaceData::Mesh::fromCharArray(data, n);
        delete [] data;
        assert(m1.equals(m2));
    }

    static void testBINZLodaderOBJWriter(const QString &srcDirPath, const QString &dstDirPath)
    {
        QDir srcDir(srcDirPath, "*.binz");
        QFileInfoList srcEntries = srcDir.entryInfoList();
        foreach (const QFileInfo &srcFileInfo, srcEntries)
        {
            Face::FaceData::Mesh srcMesh = Face::FaceData::Mesh::fromBINZ(srcFileInfo.absoluteFilePath(), true);
            QString dstFilePath = dstDirPath + QDir::separator() + srcFileInfo.baseName() + ".obj";
            qDebug() << "writing OBJ";
            srcMesh.writeOBJ(dstFilePath, '.');
        }
    }

    static int testBinReadWrite(int argc, char *argv[], const QString &dir)
    {
        Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromXYZ(dir + "xyz/02463d652.abs.xyz", false);

        mesh.writeBIN("face.bin");
        Face::FaceData::Mesh mesh2 = Face::FaceData::Mesh::fromBIN("face.bin");

        QApplication app(argc, argv);
        Face::GUI::GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh2);
        widget.show();
        return app.exec();
    }

    static int testReadBinWriteBinzReadBinz(int argc, char *argv[], const QString &dir)
    {
        Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromBIN(dir + "bin/02463d652.bin", true);
        mesh.writeBINZ("mesh.binz");
        mesh = Face::FaceData::Mesh::fromBINZ("mesh.binz");
        mesh.printStats();

        QApplication app(argc, argv);
        Face::GUI::GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh);
        widget.show();
        return app.exec();
    }

    static int readAbsWithTexture(int argc, char *argv[])
    {
        Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromABS("/run/media/stepo/My Book/3D-FRGC-data/nd1/Spring2004range/02463d652.abs",
                                                                  "/run/media/stepo/My Book/3D-FRGC-data/nd1/Spring2004range/02463d653.ppm", true);

        QApplication app(argc, argv);
        Face::GUI::GLWidget widget;
        widget.setWindowTitle("GL Widget");
        widget.addFace(&mesh);
        widget.show();
        return app.exec();
    }
};

#endif // TESTMESH_H

