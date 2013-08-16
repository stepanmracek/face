#include "filedata.h"

#include <QDebug>
#include <cassert>

int fileInfoToId(const QFileInfo &info)
{
    return info.baseName().split("d")[0].toInt();
}

FileData::FileData(const QString &path)
{
    QDir dir(path, "*.binz");
    files = dir.entryInfoList();
    qDebug() << "Loaded" << files.count() << "entries from" << path;

    int index = 0;
    foreach(const QFileInfo &info, files)
    {
        int id = fileInfoToId(info);
        if (!sameDict.contains(id)) sameDict[id] = QList<int>();
        sameDict[id] << index;
        index++;
    }

    foreach (int id, sameDict.keys())
    {
        if (sameDict[id].count() < 2) sameDict.remove(id);
    }
}

QPair<QString, QString> FileData::getNextPair(int firstIndex, int secondIndex, bool &isSamePair)
{
    QPair<QString, QString> result;
    int firstId = fileInfoToId(files[firstIndex]);
    int secondId = fileInfoToId(files[secondIndex]);
    isSamePair = (firstId == secondId);

    result.first = files[firstIndex].absoluteFilePath();
    result.second = files[secondIndex].absoluteFilePath();
    return result;
}

QPair<QString, QString> FileData::getNextPair(bool &isSamePair)
{
    int firstIndex = 0;
    int secondIndex = 0;
    int random = qrand() % 2;
    assert(random == 0 || random == 1);
    if (random)
    {
        firstIndex = qrand() % files.count();
        secondIndex = firstIndex;
        while (firstIndex == secondIndex)
        {
            secondIndex = qrand() % files.count();
        }
    }
    else
    {
        int idIndex = qrand() % sameDict.count();
        int id = sameDict.keys()[idIndex];
        const QList<int> &indicies = sameDict[id];

        firstIndex = indicies[qrand() % indicies.count()];
        secondIndex = firstIndex;
        while (firstIndex == secondIndex)
        {
            secondIndex = indicies[qrand() % indicies.count()];
        }
    }

    return getNextPair(firstIndex, secondIndex, isSamePair);
}
