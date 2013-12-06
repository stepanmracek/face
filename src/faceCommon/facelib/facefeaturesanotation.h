#ifndef FACEFEATURESANOTATION_H
#define FACEFEATURESANOTATION_H

#include <QString>

#include "mesh.h"
#include "landmarks.h"

class FaceFeaturesAnotation
{
public:  
    static void anotateBINs(const QString &dirPath, bool uniqueIDsOnly, bool overwrite);

    static Landmarks anotate(Mesh &mesh, bool &success);
};

#endif // FACEFEATURESANOTATION_H
