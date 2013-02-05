#ifndef VECTOR2_H
#define VECTOR2_H

#include <cmath>

class Vector2
{
public:
    double x;
    double y;

    Vector2()
    {
        x = 0; y = 0;
    }

    Vector2(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    double sqrMagnitude()
    {
        return (x*x)+(y*y);
    }

    double magnitude()
    {
        return sqrt(sqrMagnitude());
    }

    void normalize()
    {
        double mag = magnitude();
        x = x/mag;
        y = y/mag;
    }

    Vector2 normalized()
    {
        double mag = magnitude();
        Vector2 v(x/mag, y/mag);
        return v;
    }

    Vector2 operator+(Vector2 &b)
    {
        Vector2 v(x+b.x, y+b.y);
        return v;
    }

    Vector2 operator-(Vector2 &b)
    {
        Vector2 v(x-b.x, y-b.y);
        return v;
    }

    double operator*(Vector2 &b)
    {
        return x*b.x + y*b.y;
    }

    bool isZero()
    {
        return x==0 && y == 0;
    }
};

#endif // VECTOR2_H
