#ifndef TESTMORPHABLEFACEMODEL_H
#define TESTMORPHABLEFACEMODEL_H

#include <QApplication>
#include <QVector>
#include <QFileInfo>
#include <QDir>
#include <QSet>

#include "facelib/surfaceprocessor.h"
#include "facelib/glwidget.h"
#include "facelib/facefeaturesanotation.h"
//#include "kinect/kinnect.h"

class TestMorphableFaceModel
{
public:
    /*static int testMorphFromKinect(int argc, char *argv[], const QString &pca, const QString &flags, const QString &landmarksPath)
    {
        QApplication app(argc, argv);
        Morphable3DFaceModel model(pca, flags, landmarksPath, 200);

        qDebug() << "Modes:" << model.pca.getModes();
        model.pca.modesSelectionThreshold(0.98);
        qDebug() << "Modes:" << model.pca.getModes();

        Mesh mesh = Kinect::scanFace();
        SurfaceProcessor::smooth(mesh, 1, 5);
        Landmarks landmarks = FaceFeaturesAnotation::anotate(mesh);

        model.align(mesh, landmarks, 10);
        model.morphModel(mesh);
        mesh.move(cv::Point3d(100,0,0));

        GLWidget widget;
        widget.addFace(&(model.mesh));
        widget.addFace(&mesh);
        widget.show();

        return app.exec();
    }*/

    static void testCreate(const QString &dirPath, const QString &meanLadmarksFile,
                           const QString &pcaForZcoordFile, const QString &pcaForTextureFile,
                           const QString &pcaFile, const QString &flagsFile)
    {
        Face::FaceData::Map mask(200,200);
        cv::Point f1(100, 62);
        cv::Point f2(100, 112);
        for (int y = 0; y < 200; y++)
        {
            for (int x = 0; x < 200; x++)
            {
                cv::Point p(x, y);
                double d = Face::LinAlg::euclideanDistance(p, f1) + Face::LinAlg::euclideanDistance(p, f2);
                if (d < 125)
                {
                    mask.set(x, y, 0);
                }
            }
        }

        QVector<Face::FaceData::Mesh> meshesVec;
        QVector<Face::FaceData::VectorOfPoints> landmarksVec;
        QSet<QString> usedIDs;

        QDir dir(dirPath, "*.xml");
        QFileInfoList entries = dir.entryInfoList();
        foreach (const QFileInfo &fileInfo, entries)
        {
            qDebug() << fileInfo.baseName();
            QString id = fileInfo.baseName().mid(0, 5);
            if (usedIDs.contains(id)) continue;
            usedIDs.insert(id);

            Face::FaceData::Landmarks landmarks(fileInfo.absoluteFilePath());
            assert(landmarks.check());
            Face::FaceData::Mesh mesh = Face::FaceData::Mesh::fromBIN(dirPath + QDir::separator() + fileInfo.baseName() + ".bin");

            mesh.translate(-landmarks.get(Face::FaceData::Landmarks::Nosetip));
            Face::LinAlg::Procrustes3D::translate(landmarks.points, -landmarks.get(Face::FaceData::Landmarks::Nosetip));

            landmarksVec.append(landmarks.points);
            meshesVec.append(mesh);
        }

        //Morphable3DFaceModel::create(meshesVec, landmarksVec, 1, pcaForZcoordFile, pcaForTextureFile, pcaFile,
        //                             flagsFile, meanLadmarksFile, mask, false, false);
    }
};

#endif // TESTMORPHABLEFACEMODEL_H
