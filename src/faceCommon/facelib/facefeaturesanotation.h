#ifndef FACEFEATURESANOTATION_H
#define FACEFEATURESANOTATION_H

#include <QString>

#include "mesh.h"
#include "landmarks.h"

class FaceFeaturesAnotation
{
public:  
    static void anotateXYZ(const QString &dirPath, bool uniqueIDsOnly);

    static Landmarks anotate(Mesh &mesh, int desiredLandmarksCount, bool &success);
};

#endif // FACEFEATURESANOTATION_H
