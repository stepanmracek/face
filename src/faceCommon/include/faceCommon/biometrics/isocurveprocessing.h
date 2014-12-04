#ifndef ISOCURVEPROCESSING_H
#define ISOCURVEPROCESSING_H

#include "faceCommon/facedata/mesh.h"
#include "faceCommon/biometrics/template.h"

namespace Face {
namespace Biometrics {

typedef std::vector<FaceData::VectorOfPoints> VectorOfCurves;

struct SubjectIsoCurves
{
    int subjectID;
    VectorOfCurves vectorOfIsocurves;
};

class IsoCurveProcessing
{
public:
    IsoCurveProcessing();

    static std::vector<SubjectIsoCurves> readDirectory(const std::string &path, const std::string &separator, const std::string &fileNameFilter = "*");

    static void sampleIsoCurves(std::vector<SubjectIsoCurves> &data, int modulo);

    static void sampleIsoCurvePoints(std::vector<SubjectIsoCurves> &data, int modulo);

    static void sampleIsoCurvePoints(VectorOfCurves &isocurves, int modulo);

    static void sampleIsoCurvePoints(FaceData::VectorOfPoints &isocurve, int modulo);

    static void selectIsoCurves(std::vector<SubjectIsoCurves> &data, int start, int end);

    static void stats(std::vector<SubjectIsoCurves> &data);

    static std::vector<Template> generateTemplates(std::vector<SubjectIsoCurves> &data, bool onlyZ);

    static Template generateTemplate(const SubjectIsoCurves &subj, bool onlyZ);

    static Face::LinAlg::Vector generateFeatureVector(const VectorOfCurves &isocurves, bool onlyZ);

    static std::vector<Template> generateEuclDistanceTemplates(std::vector<SubjectIsoCurves> &data);
};

}
}

#endif // ISOCURVEPROCESSING_H
