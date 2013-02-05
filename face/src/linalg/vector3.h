#ifndef VECTOR3_H
#define VECTOR3_H

#include <cmath>

class Vector3
{
public:
    double x;
    double y;
    double z;

    Vector3()
    {
        x = 0; y = 0; z = 0;
    }

    Vector3(double x, double y, double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    double sqrMagnitude()
    {
        return (x*x)+(y*y)+(z*z);
    }

    double magnitude()
    {
        return sqrt(sqrMagnitude());
    }

    void Normalize()
    {
        double mag = magnitude();
        x = x/mag;
        y = y/mag;
        z = z/mag;
    }

    Vector3 Normalized()
    {
        double mag = magnitude();
        Vector3 v(x/mag, y/mag, z/mag);
        return v;
    }

    Vector3 operator+(Vector3 &b)
    {
        Vector3 v(x+b.x, y+b.y, z+b.z);
        return v;
    }

    Vector3 operator-(Vector3 &b)
    {
        Vector3 v(x-b.x, y-b.y, z-b.z);
        return v;
    }

    double operator*(Vector3 &b)
    {
        return x*b.x + y*b.y + z*b.z;
    }

    bool isZero()
    {
        return x==0 && y==0 && z==0;
    }
};

#endif // VECTOR3_H
