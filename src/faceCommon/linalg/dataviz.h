#ifndef DATAVIZ_H
#define DATAVIZ_H

#include <QVector>

#include "common.h"

class DataViz
{
public:
    static void ToGnuplotFile(QVector<Matrix> &vectors, const QString &path);
};

#endif // DATAVIZ_H
