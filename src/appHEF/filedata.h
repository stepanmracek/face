#ifndef FILEDATA_H
#define FILEDATA_H

#include <QString>
#include <QDir>
#include <QFileInfoList>
#include <QFileInfo>
#include <QMap>
#include <QList>
#include <QPair>

class FileData
{
private:
    QFileInfoList files;

    // translates subject id to indicies within list of files
    QMap<int, QList<int> > sameDict;

    QPair<QString, QString> getNextPair(int firstIndex, int secondIndex, bool &isSamePair);

public:
    FileData(const QString &path);

    QPair<QString, QString> getNextPair(bool &isSamePair);
};

#endif // FILEDATA_H
