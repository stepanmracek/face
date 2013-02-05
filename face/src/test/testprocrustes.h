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
        Matrix v = Vector::fromTwoColsFile("vectorsToAlign/vec01");
        Procrustes::centralize(v);
        double sum = 0.0;
        for (int i = 0; i < v.rows; i++)
            sum += v(i, 0);
        qDebug() << sum;
        Vector::toFileTwoCols(v, "original");

        RotateAndScaleCoefs rs(1.2, 0.75);
        Procrustes::rotateAndScale(v, rs);
        Vector::toFileTwoCols(v, "rotatedAndScaled");
    }

    static void testAlign()
    {
        // Load vectors
        Matrix from = Vector::fromTwoColsFile("vectorsToAlign/vec01");
        Procrustes::centralize(from);
        Matrix to = from.clone();

        // Rotate first one
        RotateAndScaleCoefs testCoefs(1.2, -0.95);
        Procrustes::rotateAndScale(to, testCoefs);

        Matrix diff = from - to;
        qDebug() << "before:" << Vector::sqrMagnitude(diff);
        Vector::toFileTwoCols(from, "before");
        Vector::toFileTwoCols(to, "before", true);

        // Estimate needed rotation of second one
        //TransformationCoefs gainedCoefs = Procrustes::AlignAlt(from, to);
        RotateAndScaleCoefs gainedCoefs = Procrustes::align(from, to);
        //Procrustes::Transformate(from, gainedCoefs);
        Procrustes::rotateAndScale(from, gainedCoefs);
        qDebug() << "coefs:" << gainedCoefs.s << gainedCoefs.theta;
        diff = from - to;
        qDebug() << "after:" << Vector::sqrMagnitude(diff);

        Vector::toFileTwoCols(from, "after");
        Vector::toFileTwoCols(to, "after", true);
    }

    static void testProcrustes()
    {
        // load vectors
        QVector<Matrix> vectors;
        QDir dir("vectorsToAlign");
        dir.setFilter(QDir::Files | QDir::NoDotAndDotDot);
        QStringList filenames = dir.entryList();

        for (int i = 0; i < filenames.count(); i++)
        {
            Matrix v = Vector::fromTwoColsFile("vectorsToAlign/" + filenames[i]);
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
            Vector::toFileTwoCols(vectors[i], "unaligned", true);
        }

        // align them
        Procrustes::procrustesAnalysis(vectors, 1e-100);

        // save all to one file
        if (QFile::exists("aligned"))
            QFile::remove("aligned");

        for (int i = 0; i < vectors.count(); i++)
        {
            Vector::toFileTwoCols(vectors[i], "aligned", true);
        }
    }
};

#endif // TESTPROCRUSTES_H
