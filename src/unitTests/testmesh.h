#ifndef TESTMESH_H
#define TESTMESH_H

#include <QString>
#include <QDir>
#include <QFileInfoList>

#include "facelib/mesh.h"

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
};

#endif // TESTMESH_H
