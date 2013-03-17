#ifndef VOSMFACADE_H
#define VOSMFACADE_H

#include <QString>

enum ModelType
{

};

class VOSMFacade
{
public:
    VOSMFacade();

    bool learn(const QString &shapeInfoFilePath, const QString &imageDirPath, const QString &imageNameFilter,
               const QString &anotationsDirPath, const QString &outputDirPath);

    bool fit();
};

#endif // VOSMFACADE_H
