#ifndef TESTPROCRUSTES_H
#define TESTPROCRUSTES_H

#include <QVector>
#include <QDir>
#include <QStringList>
#include <QDebug>
#include <QFile>

#include "linalg/vector.h"
#include "linalg/procrustes.h"

class TestProcrustes
{
public:
    static void testAll()
    {
        testAlign();
        testProcrustes();
    }

    static void testRotateAndScale()
    {
        Face::LinAlg::Vector v = Face::LinAlg::Vector::fromTwoColsFile("vectorsToAlign/vec01");
        Face::LinAlg::Procrustes2D::centralize(v);
        double sum = 0.0;
        for (int i = 0; i < v.rows; i++)
            sum += v(i, 0);
        qDebug() << sum;
        v.toFileTwoCols("original");

        Face::LinAlg::RotateAndScaleCoefs rs(1.2, 0.75);
        Face::LinAlg::Procrustes2D::rotateAndScale(v, rs);
        v.toFileTwoCols("rotatedAndScaled");
    }

    static void testAlign()
    {
        // Load vectors
        Face::LinAlg::Vector from = Face::LinAlg::Vector::fromTwoColsFile("vectorsToAlign/vec01");
        Face::LinAlg::Procrustes2D::centralize(from);
        Face::LinAlg::Vector to(from);

        // Rotate first one
        Face::LinAlg::RotateAndScaleCoefs testCoefs(1.2, -0.95);
        Face::LinAlg::Procrustes2D::rotateAndScale(to, testCoefs);

        Face::LinAlg::Vector diff = from - to;
        qDebug() << "before:" << diff.sqrMagnitude();
        from.toFileTwoCols("before");
        to.toFileTwoCols("before", true);

        // Estimate needed rotation of second one
        //TransformationCoefs gainedCoefs = Procrustes::AlignAlt(from, to);
        Face::LinAlg::RotateAndScaleCoefs gainedCoefs = Face::LinAlg::Procrustes2D::align(from, to);
        //Procrustes::Transformate(from, gainedCoefs);
        Face::LinAlg::Procrustes2D::rotateAndScale(from, gainedCoefs);
        qDebug() << "coefs:" << gainedCoefs.s << gainedCoefs.theta;
        diff = from - to;
        qDebug() << "after:" << diff.sqrMagnitude();

        from.toFileTwoCols("after");
        to.toFileTwoCols("after", true);
    }

    static void testProcrustes()
    {
        // load vectors
        QVector<Face::LinAlg::Vector> vectors;
        QDir dir("vectorsToAlign");
        dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);
        QStringList filenames = dir.entryList();

        for (int i = 0; i < filenames.count(); i++)
        {
            Face::LinAlg::Vector v = Face::LinAlg::Vector::fromTwoColsFile("vectorsToAlign/" + filenames[i]);
            Face::LinAlg::RotateAndScaleCoefs c;
            c.s = 1;
            c.theta = ((double)qrand())/RAND_MAX - 0.5;
            Face::LinAlg::Procrustes2D::rotateAndScale(v, c);
            vectors.append(v);
        }

        if (QFile::exists("unaligned"))
            QFile::remove("unaligned");

        for (int i = 0; i < vectors.count(); i++)
        {
            vectors[i].toFileTwoCols("unaligned", true);
        }

        // align them
        Face::LinAlg::Procrustes2D::procrustesAnalysis(vectors, 1e-100);

        // save all to one file
        if (QFile::exists("aligned"))
            QFile::remove("aligned");

        for (int i = 0; i < vectors.count(); i++)
        {
            vectors[i].toFileTwoCols("aligned", true);
        }
    }
};

#endif // TESTPROCRUSTES_H
