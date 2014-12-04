#include "faceCommon/biometrics/isocurveprocessing.h"

#include <Poco/NumberParser.h>
#include <Poco/StringTokenizer.h>
#include <Poco/Path.h>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/linalg/serialization.h"
#include "faceCommon/facedata/surfaceprocessor.h"

using namespace Face::Biometrics;

IsoCurveProcessing::IsoCurveProcessing()
{
}

std::vector<SubjectIsoCurves> IsoCurveProcessing::readDirectory(const std::string &path, const std::string &separator, const std::string &fileNameFilter)
{
    auto files = Face::LinAlg::Loader::listFiles(path, fileNameFilter, Face::LinAlg::Loader::Filename);

    std::vector<SubjectIsoCurves> result;
    for(const auto &file : files)
    {
        SubjectIsoCurves subject;

        subject.subjectID = Poco::NumberParser::parse(Poco::StringTokenizer(file, separator)[0]);
        subject.vectorOfIsocurves = Face::LinAlg::Serialization::readVectorOfPointclouds(path + Poco::Path::separator() + file);

        result.push_back(subject);
    }

    return result;
}

void IsoCurveProcessing::sampleIsoCurves(std::vector<SubjectIsoCurves> &data, int modulo)
{
    for (unsigned int i = 0; i < data.size(); i++)
    {
        SubjectIsoCurves &subj = data[i];
        VectorOfCurves newIsoCurves;
        for (unsigned int j = 0; j < subj.vectorOfIsocurves.size(); j += modulo)
        {
            newIsoCurves.push_back(subj.vectorOfIsocurves[j]);
        }
        subj.vectorOfIsocurves = newIsoCurves;
    }
}


void IsoCurveProcessing::sampleIsoCurvePoints(std::vector<SubjectIsoCurves> &data, int modulo)
{
    for (unsigned int i = 0; i < data.size(); i++)
    {
        SubjectIsoCurves &subj = data[i];
        sampleIsoCurvePoints(subj.vectorOfIsocurves, modulo);
        /*VectorOfIsocurves newIsoCurves;
        for (int j = 0; j < subj.vectorOfIsocurves.count(); j++)
        {
            VectorOfPoints &isocurve = subj.vectorOfIsocurves[j];
            VectorOfPoints newIsoCurve;
            for (int k = 0; k < isocurve.count(); k += modulo)
            {
                newIsoCurve << isocurve[k];
            }
            newIsoCurves << newIsoCurve;
        }
        subj.vectorOfIsocurves = newIsoCurves;*/
    }
}

void IsoCurveProcessing::sampleIsoCurvePoints(VectorOfCurves &isocurves, int modulo)
{
    for (unsigned int i = 0; i < isocurves.size(); i++)
    {
        sampleIsoCurvePoints(isocurves[i], modulo);
    }
}

void IsoCurveProcessing::sampleIsoCurvePoints(Face::FaceData::VectorOfPoints &isocurve, int modulo)
{
    Face::FaceData::VectorOfPoints newIsoCurve;
    for (unsigned int i = 0; i < isocurve.size(); i += modulo)
    {
        newIsoCurve.push_back(isocurve[i]);
    }
    isocurve = newIsoCurve;
}

void IsoCurveProcessing::selectIsoCurves(std::vector<SubjectIsoCurves> &data, int start, int end)
{
    for (unsigned int i = 0; i < data.size(); i++)
    {
        SubjectIsoCurves &subj = data[i];
        VectorOfCurves newIsoCurves;
        for (int j = start; j < end; j++)
        {
            newIsoCurves.push_back(subj.vectorOfIsocurves[j]);
        }
        subj.vectorOfIsocurves = newIsoCurves;
    }
}

void IsoCurveProcessing::stats(std::vector<SubjectIsoCurves> &data)
{
    if ((data.size() == 0) ||
        (data[0].vectorOfIsocurves.size() == 0) ||
        (data[0].vectorOfIsocurves[0].size() == 0))
        throw FACELIB_EXCEPTION("invalid input data");

    int samplesCount = data[0].vectorOfIsocurves[0].size();
    int curvesCount = data[0].vectorOfIsocurves.size();
    int subjectCount = data.size();

    for (int curveIndex = 0; curveIndex < curvesCount; curveIndex++)
    {
        int nanCount = 0;
        for (int subjectIndex = 0; subjectIndex < subjectCount; subjectIndex++)
        {
            for (int sampleIndex = 0; sampleIndex < samplesCount; sampleIndex++)
            {
                cv::Point3d &p = data[subjectIndex].vectorOfIsocurves[curveIndex][sampleIndex];
                if (p.x != p.x || p.y != p.y || p.z != p.z)
                {
                    nanCount += 1;
                }
            }
            /*if (nanCount > 0)
            {
                qDebug() << data[subjectIndex].subjectID;
            }*/
        }

        std::cout << "curveIndex: " << curveIndex << " All samples valid: " << (nanCount == 0) << std::endl;
    }
}

std::vector<Template> IsoCurveProcessing::generateTemplates(std::vector<SubjectIsoCurves> &data, bool onlyZ)
{
    std::vector<Template> result;
    for (const SubjectIsoCurves &subjectIsoCurves : data)
    {
        result.push_back(generateTemplate(subjectIsoCurves, onlyZ));
    }

    return result;
}

Template IsoCurveProcessing::generateTemplate(const SubjectIsoCurves &subj, bool onlyZ)
{
    Template t;
    t.subjectID = subj.subjectID;
    t.featureVector = generateFeatureVector(subj.vectorOfIsocurves, onlyZ);
    return t;
}

Face::LinAlg::Vector IsoCurveProcessing::generateFeatureVector(const VectorOfCurves &isocurves, bool onlyZ)
{
    std::vector<double> fv;
    for (const FaceData::VectorOfPoints &isocurve : isocurves)
    {
        for (const cv::Point3d &p : isocurve)
        {
            if (!onlyZ)
            {
                fv.push_back(p.x);
                fv.push_back(p.y);
            }
            fv.push_back(p.z);
            //qDebug() << isocurves.indexOf(isocurve) << fv.count();
        }
    }
    //exit(0);
    return Face::LinAlg::Vector(fv);
}

std::vector<Template> IsoCurveProcessing::generateEuclDistanceTemplates(std::vector<SubjectIsoCurves> &data)
{
    std::vector<Template> result;
    for (const SubjectIsoCurves &subjectIsoCurves : data)
    {
        Template t;
        t.subjectID = subjectIsoCurves.subjectID;

        std::vector<double> fv;
        for (const FaceData::VectorOfPoints &isocurve : subjectIsoCurves.vectorOfIsocurves)
        {
            const auto &v = FaceData::SurfaceProcessor::isoGeodeticCurveToEuclDistance(isocurve, cv::Point3d(0,0,0));
            fv.insert(fv.end(), v.begin(), v.end());
        }

        t.featureVector = Face::LinAlg::Vector(fv);
        result.push_back(t);
    }

    return result;
}
