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
        Vector v = Vector::fromTwoColsFile("vectorsToAlign/vec01");
        Procrustes::centralize(v);
        double sum = 0.0;
        for (int i = 0; i < v.rows; i++)
            sum += v(i, 0);
        qDebug() << sum;
        v.toFileTwoCols("original");

        RotateAndScaleCoefs rs(1.2, 0.75);
        Procrustes::rotateAndScale(v, rs);
        v.toFileTwoCols("rotatedAndScaled");
    }

    static void testAlign()
    {
        // Load vectors
        Vector from = Vector::fromTwoColsFile("vectorsToAlign/vec01");
        Procrustes::centralize(from);
        Vector to(from);

        // Rotate first one
        RotateAndScaleCoefs testCoefs(1.2, -0.95);
        Procrustes::rotateAndScale(to, testCoefs);

        Matrix diffMat = from - to;
        Vector diff = diffMat;
        qDebug() << "before:" << diff.sqrMagnitude();
        from.toFileTwoCols("before");
        to.toFileTwoCols("before", true);

        // Estimate needed rotation of second one
        //TransformationCoefs gainedCoefs = Procrustes::AlignAlt(from, to);
        RotateAndScaleCoefs gainedCoefs = Procrustes::align(from, to);
        //Procrustes::Transformate(from, gainedCoefs);
        Procrustes::rotateAndScale(from, gainedCoefs);
        qDebug() << "coefs:" << gainedCoefs.s << gainedCoefs.theta;
        diffMat = from - to;
        diff = diffMat;
        qDebug() << "after:" << diff.sqrMagnitude();

        from.toFileTwoCols("after");
        to.toFileTwoCols("after", true);
    }

    static void testProcrustes()
    {
        // load vectors
        QVector<Vector> vectors;
        QDir dir("vectorsToAlign");
        dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);
        QStringList filenames = dir.entryList();

        for (int i = 0; i < filenames.count(); i++)
        {
            Vector v = Vector::fromTwoColsFile("vectorsToAlign/" + filenames[i]);
            RotateAndScaleCoefs c;
            c.s = 1;
            c.theta = ((double)qrand())/RAND_MAX - 0.5;
            Procrustes::rotateAndScale(v, c);
            vectors.append(v);
        }

        if (QFile::exists("unaligned"))
            QFile::remove("unaligned");

        for (int i = 0; i < vectors.count(); i++)
        {
            vectors[i].toFileTwoCols("unaligned", true);
        }

        // align them
        Procrustes::procrustesAnalysis(vectors, 1e-100);

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
