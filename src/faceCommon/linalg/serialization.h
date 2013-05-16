#ifndef SERIALIZATION_H
#define SERIALIZATION_H

#include <QVector>
#include <QString>

#include "common.h"
#include "facelib/mesh.h"

class Serialization
{
public:
    static void serializeVectorOfPointClouds(QVector<VectorOfPoints> &data, const QString &path);
};

#endif // SERIALIZATION_H
