#ifndef TESTANOTATION_H
#define TESTANOTATION_H

#include <QString>

#include "facedata/facefeaturesanotation.h"

class TestAnotation
{
public:
    static void test(const QString &path)
    {
        Face::FaceData::FaceFeaturesAnotation::anotateBINs(path, true, false);
    }
};

#endif // TESTANOTATION_H
