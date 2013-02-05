#ifndef FACEFEATURESANOTATION_H
#define FACEFEATURESANOTATION_H

#include <QString>

#include "mesh.h"
#include "landmarks.h"

class FaceFeaturesAnotation
{
public:  
    static void anotateOBJ(const QString &dirPath, bool uniqueIDsOnly);

    static Landmarks anotate(Mesh &mesh);
};

#endif // FACEFEATURESANOTATION_H
