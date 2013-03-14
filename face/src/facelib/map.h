#ifndef MAP_H
#define MAP_H

#include <QDebug>

#include <cmath>
#include <opencv/cv.h>

#include "maskedvector.h"
#include "linalg/common.h"

class Map
{
public:

    QVector<double> values;
    QVector<bool> flags;
    int w;
    int h;

    Map();
    void init(int _w, int _h);
    Map(int w, int h);
    //virtual ~Map();

    QVector<double> getUsedValues() const;

    int coordToIndex(int x, int y) const
    {
        return y*w + x;
    }

    void set(int x, int y, double v)
    {
        int i = coordToIndex(x, y);
        flags[i] = true;
        values[i] = v;
    }

    void set(int i, double v)
    {

        flags[i] = true;
        values[i] = v;
    }

    void setAll(double v);

    void unset(int x, int y)
    {
        int i = coordToIndex(x, y);
        unset(i);
    }

    void unset(int i)
    {
        flags[i] = false;
        values[i] = 0.0;
    }

    void unsetAll();

    bool isSet(int x, int y) const
    {
        if (!isValidCoord(x, y)) return false;
        return flags[coordToIndex(x, y)];
    }

    double get(int x, int y) const
    {
        int i = coordToIndex(x, y);
        assert(isValidCoord(x,y));
        assert(flags[i]);
        return values[i];
    }

    bool has8neigbours(int x, int y) const
    {
        /*int x = i % w;
        int y = i / w;*/

        if (x == 0 || x >= (w-1) || y == 0 || y >= (h-1)) return false;

        if (!flags[coordToIndex(x+1,y  )]) return false;
        if (!flags[coordToIndex(x,  y+1)]) return false;
        if (!flags[coordToIndex(x+1,y+1)]) return false;
        if (!flags[coordToIndex(x-1,y  )]) return false;
        if (!flags[coordToIndex(x,  y-1)]) return false;
        if (!flags[coordToIndex(x-1,y-1)]) return false;
        if (!flags[coordToIndex(x-1,y+1)]) return false;
        if (!flags[coordToIndex(x+1,y-1)]) return false;

        return true;
    }

    bool isValidCoord(int x, int y) const
    {
        return (x < w) && (x >= 0) && (y < h) && (y >= 0);
    }

    void levelSelect(double zLevel);

    void erode(int kernelSize);

    void linearScale(double multiply, double add);

    double minValue() const;

    double maxValue() const;

    void add(Map &other);

    void linearTransform(double multiply, double add);

    MaskedVector horizontalProfile(int y) const;

    MaskedVector verticalProfile(int x) const;

    MaskedVector meanVerticalProfile() const;

    MaskedVector maxVerticalProfile() const;

    MaskedVector medianVerticalProfile() const;

    MaskedVector horizontalPointDensity(int y, int stripeWidth) const;

    Map densityMap(int windowSize, bool fromCenter) const;

    int maxIndex() const;

    int indexToX(int i) const
    {
        return i % w;
    }

    int indexToY(int i) const
    {
        return i / w;
    }

    Matrix toMatrix(double voidValue = 0.0, double min = 0.0, double max = 0.0) const;

    void getCropParams(int &startx, int &width, int &starty, int &height) const;

    Map subMap(int startx, int width, int starty, int height) const;

    static Map fromMatrix(Matrix &matrix, double voidValue = 0.0);
};

#endif // MAP_H
