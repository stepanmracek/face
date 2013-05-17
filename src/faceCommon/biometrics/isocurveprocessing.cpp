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
