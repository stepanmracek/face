#include "isocurveprocessing.h"

#include <QDir>
#include <QFileInfoList>

#include "linalg/serialization.h"

IsoCurveProcessing::IsoCurveProcessing()
{
}

QVector<SubjectIsoCurves> IsoCurveProcessing::readDirectory(const QString &path, const QString &separator, const QString &fileNameFilter)
{
    QDir dir(path, fileNameFilter);
    QFileInfoList files = dir.entryInfoList();

    QVector<SubjectIsoCurves> result;
    foreach(const QFileInfo &fileInfo, files)
    {
        SubjectIsoCurves subject;

        subject.id = fileInfo.baseName().split(separator)[0];
        subject.vectorOfIsocurves = Serialization::readVectorOfPointclouds(fileInfo.absoluteFilePath());

        result << subject;
    }

    return result;
}

void IsoCurveProcessing::sampleIsoCurves(QVector<SubjectIsoCurves> &data, int modulo)
{
    for (int i = 0; i < data.count(); i++)
    {
        SubjectIsoCurves &subj = data[i];
        VectorOfIsocurves newIsoCurves;
        for (int j = 0; j < subj.vectorOfIsocurves.count(); j += modulo)
        {
            newIsoCurves << subj.vectorOfIsocurves[j];
        }
        subj.vectorOfIsocurves = newIsoCurves;
    }
}


void IsoCurveProcessing::sampleIsoCurvePoints(QVector<SubjectIsoCurves> &data, int modulo)
{
    for (int i = 0; i < data.count(); i++)
    {
        SubjectIsoCurves &subj = data[i];
        VectorOfIsocurves newIsoCurves;
        for (int j = 0; j < subj.vectorOfIsocurves.count(); j++)
        {
            VectorOfPoints &isocurve = subj.vectorOfIsocurves[j];
            VectorOfPoints newIsoCurve;
            for (int k = 0; k < isocurve.count(); k += modulo)
            {
                newIsoCurve << isocurve[k];
            }
            newIsoCurves << newIsoCurve;
        }
        subj.vectorOfIsocurves = newIsoCurves;
    }
}
