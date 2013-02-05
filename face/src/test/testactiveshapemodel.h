#ifndef TESTACTIVESHAPEMODEL_H
#define TESTACTIVESHAPEMODEL_H

#include <QVector>
#include <QDir>
#include <QDebug>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "linalg/vector.h"
#include "linalg/pca.h"
#include "amlib/activeshapemodel.h"
#include "linalg/matrixconverter.h"
#include "linalg/loader.h"

class TestActiveShapeModel
{
public:
    static void testLearn()
    {
        // load shapes
        QString path("testASM");
        QVector<Matrix> shapes = Loader::loadShapes(path);
        //cv::Mat testShape = shapes.at(0).clone();

        // load images
        QVector<Matrix> images = Loader::loadImages(path);

        // train model
        ActiveShapeModelSettings settings;
        settings.neighborhoodSize = 5;
        settings.points.fromFile("testASM.model");
        ActiveShapeModel model(shapes, images, settings);

        // output some results
        // aligned shapes
        QString alignedShapes("alignedShapes");
        if (QFile::exists(alignedShapes))
            QFile::remove(alignedShapes);
        for (int i = 0; i < shapes.count(); i++)
            Vector::toFileTwoCols(shapes[i], alignedShapes, true);

        // first 5 modes of pca
        for (int mode = 0; mode < 5; mode++)
        {
            QString modeFile("mode");
            modeFile.append(QString::number(mode));
            if (QFile::exists(modeFile))
                QFile::remove(modeFile);

            double maxb = model.b(mode);
            double bDelta = (maxb - (-maxb))/10.0;
            for (double b = -maxb; b <= maxb; b += bDelta)
            {
                Matrix projected = Matrix::zeros(model.pca.cvPca.eigenvalues.rows, 1);
                projected(mode, 0) = b;
                Matrix backProjected = model.pca.backProject(projected);
                Vector::toFileTwoCols(backProjected, modeFile, true);
            }
        }
    }

    static void testInstantiate()
    {
        // load shapes
        QString path("testASM");
        QVector<Matrix> shapes = Loader::loadShapes(path);
        Matrix testShape = shapes.at(0).clone();

        // load images
        QVector<Matrix> images = Loader::loadImages(path);

        // train model
        ActiveShapeModelSettings settings;
        settings.neighborhoodSize = 5;
        settings.points.fromFile("testASM.model");
        ActiveShapeModel model(shapes, images, settings);

        // draw first
        model.drawModelInstance(images[0], testShape, 1);
        cv::imshow("ASM", images[0]);
        cv::waitKey();
    }

    static void testFitToShape()
    {
        // load shapes
        QString path("testASM");
        QVector<Matrix> shapes = Loader::loadShapes(path);
        Matrix testShape = shapes.at(0).clone();

        // load images
        QVector<Matrix> images = Loader::loadImages(path);

        // train model
        ActiveShapeModelSettings settings;
        settings.neighborhoodSize = 5;
        settings.enhancedSearchSize = 5;
        settings.points.fromFile("testASM.model");
        ActiveShapeModel model(shapes, images, settings);

        // fit first
        TranslationCoefs t;
        RotateAndScaleCoefs rs;
        Matrix B = model.FitToShape(testShape, t, rs, 1e-30);

        Matrix resultShape = model.pca.backProject(B);
        Procrustes::rotateAndScale(resultShape, rs);
        Procrustes::translate(resultShape, t);

        // display result
        Matrix blank = Matrix::ones(images.at(0).rows, images.at(0).cols);
        model.drawModelInstance(blank, testShape, 0.5);
        model.drawModelInstance(blank, resultShape, 0);
        cv::imshow("Fit", blank);
        cv::waitKey();
    }

    static void testFitToShapeIterative()
    {
        // load shapes
        QString path("/home/stepo/SVN/disp-stepan-mracek/test/testASM");
        QVector<Matrix> shapes = Loader::loadShapes(path);
        Matrix testShape = shapes.at(0).clone();

        // load images
        QVector<Matrix> images = Loader::loadImages(path);

        // train model
        ActiveShapeModelSettings settings;
        settings.neighborhoodSize = 5;
        settings.enhancedSearchSize = 5;
        settings.points.fromFile("/home/stepo/SVN/disp-stepan-mracek/test/testASM.model");
        ActiveShapeModel model(shapes, images, settings);

        for (int i = 0; i < 50000; i++)
        {
            // fit first
            TranslationCoefs t;
            RotateAndScaleCoefs rs;
            Matrix B = model.FitToShape(testShape, t, rs, 1e-30);

            Matrix resultShape = model.pca.backProject(B);
            Procrustes::rotateAndScale(resultShape, rs);
            Procrustes::translate(resultShape, t);

            // display result
            Matrix blank = Matrix::ones(images.at(0).rows, images.at(0).cols);
            model.drawModelInstance(blank, resultShape, 0);
            cv::imshow("Fit", blank);
            cv::waitKey(1);

            qDebug() << i << "/" << 50000;

            testShape = resultShape.clone();
        }
    }

    static void testFitToImage()
    {
        // load shapes
        QString path("/home/stepo/SVN/disp-stepan-mracek/test/testASM");
        QVector<Matrix> shapes = Loader::loadShapes(path);
        Matrix testShape = shapes.at(0).clone();

        // load images
        QVector<Matrix> images = Loader::loadImages(path);

        // train model
        ActiveShapeModelSettings settings;
        settings.neighborhoodSize = 15;
        settings.enhancedSearchSize = 30;
        settings.points.fromFile("/home/stepo/SVN/disp-stepan-mracek/test/testASM.model");
        ActiveShapeModel model(shapes, images, settings);

        // Set initial position
        RotateAndScaleCoefs testRs(1.02, 0.1);
        Procrustes::rotateAndScale(testShape, testRs);
        TranslationCoefs testT(20, -30);
        Procrustes::translate(testShape, testT);

        // fit
        TranslationCoefs t;
        RotateAndScaleCoefs rs;
        Matrix b = model.FitToImage(testShape, images[0], t, rs, 1e-30);

        Matrix resultShape = model.pca.backProject(b);
        Procrustes::rotateAndScale(resultShape, rs);
        Procrustes::translate(resultShape, t);

        // display result
        //model.drawModelInstance(images[0], resultShape, 1);
        //model.drawModelInstance(images[0], testShape, 0.5);
        //cv::imshow("Fit", images[0]);
        //cv::waitKey();
    }
};

#endif // TESTACTIVESHAPEMODEL_H
