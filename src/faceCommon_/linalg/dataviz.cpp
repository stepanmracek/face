#include "dataviz.h"

#include <QFile>
#include <QIODevice>
#include <QTextStream>

void DataViz::ToGnuplotFile(QVector<Matrix> &vectors, const QString &path)
{
    int n = vectors.count();
    if (n == 0) return;
    int m = vectors[0].rows;

    QFile f(path);
    f.open(QIODevice::WriteOnly);
    QTextStream out(&f);

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < m; j++)
        {
            out << vectors[i](j) << " ";
        }
        out << '\n';
    }

    out.flush();
    f.close();
}
