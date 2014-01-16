#ifndef TESTVECTOR_H
#define TESTVECTOR_H

#include <QDebug>

#include <cassert>

#include "linalg/vector.h"
#include "linalg/common.h"

class TestVector
{
private:
    static void normalize(Face::LinAlg::Vector &v)
    {
        v.normalize();
    }

public:
    static void testConstructor()
    {
        Face::LinAlg::Vector vec(3); vec(0) = 1; vec(1) = 2; vec(2) = 3;
        Face::LinAlg::Vector &vec2 = vec;
        Face::LinAlg::Vector vec3 = vec;
        normalize(vec);
        Face::LinAlg::Common::printMatrix(vec);
        Face::LinAlg::Common::printMatrix(vec2);
        Face::LinAlg::Common::printMatrix(vec3);
    }

    static void testBasicOperations()
    {
        Face::LinAlg::Vector vec1(3); vec1(0) = 1; vec1(1) = 2; vec1(2) = 3;
        Face::LinAlg::Vector vec2(3); vec2(0) = 3; vec2(1) = 2; vec2(2) = 1;
        Face::LinAlg::Vector vec3 = vec1 + vec2;
        Face::LinAlg::Common::printMatrix(vec1);
        Face::LinAlg::Common::printMatrix(vec2);
        Face::LinAlg::Common::printMatrix(vec3);
    }
};

#endif // TESTVECTOR_H
