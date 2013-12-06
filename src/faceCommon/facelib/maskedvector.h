#ifndef VECTOR_H
#define VECTOR_H

#include <QVector>

//typedef QVector<double> Vector;

class MaskedVector
{
public:
    int n;
    QVector<bool> flags;
    QVector<double> values;

    MaskedVector(int len, double initValue, bool initFlags)
    {
        n = len;
        flags = QVector<bool>(len, initFlags);
        values = QVector<double>(len, initValue);
    }

    static MaskedVector diff(MaskedVector &a, MaskedVector &b);

    /*virtual ~MaskedVector()
    {
        delete [] flags;
        delete [] values;
    }*/

    void set(int i, double value)
    {
        flags[i] = true;
        values[i] = value;
    }

    int flagCount()
    {
        int count = 0;
        for (int i = 0; i < n; i++)
        {
            if (flags[i])
            {
                count++;
            }
        }
        return count;
    }

    MaskedVector derivate();

    bool save(const char * filename);
    bool savePlot(const char * filename, bool append = false);

    int maxIndex();
    double mean();
    double max();
    double median();
};

#endif // VECTOR_H
