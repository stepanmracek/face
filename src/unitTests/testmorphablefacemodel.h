#ifndef TESTMORPHABLEFACEMODEL_H
#define TESTMORPHABLEFACEMODEL_H

#include <QApplication>
#include <QVector>
#include <QFileInfo>
#include <QDir>
#include <QSet>

#include "facelib/morphable3dfacemodelwidget.h"
#include "facelib/morphable3dfacemodel.h"
#include "facelib/surfaceprocessor.h"
#include "facelib/glwidget.h"
#include "facelib/facefeaturesanotation.h"
//#include "kinect/kinnect.h"

class TestMorphableFaceModel
{
public:

    static int testModel(int argc, char *argv[], const QString &pcaForZcoord, const QString &pcaForTexture,
                         const QString &flags, const QString &landmarksPath)
    {
        //Morphable3DFaceModel model(pca, flags, 200);
        Morphable3DFaceModel model(pcaForZcoord, pcaForTexture, flags, landmarksPath, 200);

        QApplication app(argc, argv);
        Morphable3DFaceModelWidget widget;
        widget.setModel(&model);
        widget.setWindowTitle("GL Widget");
        widget.show();

        return app.exec();
    }

    static int testMorph(int argc, char *argv[], const QString &pcaForZcoord, const QString &pcaForTexture,
                         const QString &flags, const QString &landmarksPath, const QString &probeOBJPath)
    {
        Morphable3DFaceModel model(pcaForZcoord, pcaForTexture, flags, landmarksPath, 200);

        QFileInfo inputObjInfo(probeOBJPath);
        assert(inputObjInfo.exists());
        Mesh inputMesh = Mesh::fromOBJ(probeOBJPath);
        Landmarks inputLandmarks(inputObjInfo.absolutePath() + QDir::separator() + inputObjInfo.baseName() + ".xml");

        model.align(inputMesh, inputLandmarks, 10);
        model.morphModel(inputMesh);

        QApplication app(argc, argv);

        GLWidget widgetInput;
        widgetInput.addFace(&inputMesh);
        widgetInput.show();

        GLWidget widget;
        widget.addFace(&(model.mesh));
        widget.show();

        return app.exec();
    }

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
                           const QString &flagsFile)
    {
        Map mask(200,200);
        cv::Point f1(100, 75);
        cv::Point f2(100, 125);
        for (int y = 0; y < 200; y++)
        {
            for (int x = 0; x < 200; x++)
            {
                cv::Point p(x, y);
                double d = euclideanDistance(p, f1) + euclideanDistance(p, f2);
                if (d < 125)
                {
                    mask.set(x, y, 0);
                }
            }
        }

        QVector<Mesh> meshesVec;
        QVector<VectorOfPoints> landmarksVec;
        QSet<QString> usedIDs;

        QDir dir(dirPath, "*.xml");
        QFileInfoList entries = dir.entryInfoList();
        foreach (const QFileInfo &fileInfo, entries)
        {
            qDebug() << fileInfo.baseName();
            QString id = fileInfo.baseName().mid(0, 5);
            if (usedIDs.contains(id)) continue;
            usedIDs.insert(id);

            Landmarks landmarks(fileInfo.absoluteFilePath());
            assert(landmarks.check());
            Mesh mesh = Mesh::fromBIN(dirPath + QDir::separator() + fileInfo.baseName() + ".bin");

            landmarksVec.append(landmarks.points);
            meshesVec.append(mesh);
        }

        Morphable3DFaceModel::create(meshesVec, landmarksVec, 1, pcaForZcoordFile, pcaForTextureFile, flagsFile, meanLadmarksFile, mask);
    }
};

#endif // TESTMORPHABLEFACEMODEL_H
