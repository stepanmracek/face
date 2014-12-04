#include "faceCommon/biometrics/template.h"

#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>
#include <Poco/Path.h>
#include <fstream>

#include "faceCommon/linalg/loader.h"
#include "faceCommon/biometrics/featureextractor.h"

using namespace Face::Biometrics;

std::vector<Template> Template::loadTemplates(const std::string &dirPath, const std::string &classSeparator)
{
    std::vector<Template> templates;
    std::vector<std::string> filenames = Face::LinAlg::Loader::listFiles(dirPath, "*", Face::LinAlg::Loader::AbsoluteFull);

    for (const std::string &filename : filenames)
    {
        Face::LinAlg::Vector featureVector = Face::LinAlg::Vector::fromFile(filename);

        Poco::StringTokenizer tokenizer(filename, classSeparator);
        int subjectID = Poco::NumberParser::parse(tokenizer[0]);

        Template t;
        t.subjectID = subjectID;
        t.featureVector = featureVector;

        templates.push_back(t);
    }

    return templates;
}

void Template::normalizeFeatureVectorComponents(std::vector<Template> &templates,
                                                std::vector<double> &minValues, std::vector<double> &maxValues)
{
    for (auto i = templates.begin(); i != templates.end(); ++i)
    {
        i->featureVector = i->featureVector.normalizeComponents(minValues, maxValues);
    }
}

void Template::splitVectorsAndClasses(std::vector<Template> &templates,
                                      std::vector<Face::LinAlg::Vector> &featureVectors,
                                      std::vector<int> &classMemberships)
{
    featureVectors.clear();
    classMemberships.clear();
    for (const Template &t : templates)
    {
        featureVectors.push_back(t.featureVector);
        classMemberships.push_back(t.subjectID);
    }
}

std::vector<Template> Template::joinVectorsAndClasses(const std::vector<Face::LinAlg::Vector> &featureVectors,
                                                  const std::vector<int> &classMemberships)
{
	unsigned int n = featureVectors.size();
    if (n != classMemberships.size()) throw FACELIB_EXCEPTION("featureVectors and classMemberships count mismatch");

    std::vector<Template> templates;
    for (unsigned int i = 0; i < n; i++)
    {
        Template t;
        t.subjectID = classMemberships[i];
        t.featureVector = featureVectors[i];
        templates.push_back(t);
    }
    return templates;
}

void Template::saveTemplates(std::vector<std::vector<Template> > &clusters, const std::string &path, const std::string &classSeparator)
{
    std::map<int, int> classUsage;
    for (const std::vector<Template> &templates : clusters)
    {
        for (const Template &t : templates)
        {
            if (classUsage.count(t.subjectID) == 0)
                classUsage[t.subjectID] = 0;
            classUsage[t.subjectID] += 1;

            std::string filePath = path + Poco::Path::separator() + std::to_string(t.subjectID)
                    + classSeparator + std::to_string(classUsage[t.subjectID]);

            t.featureVector.toFile(filePath);
        }
    }
}

Face::LinAlg::ZScoreNormalizationResult Template::zScoreNorm(std::vector<Template> &templates)
{
    std::vector<Face::LinAlg::Vector> vectors = getVectors(templates);
    Face::LinAlg::ZScoreNormalizationResult result = Face::LinAlg::Normalization::zScoreNormalization(vectors);
    //zScoreNorm(templates, result);
    return result;
}

void Template::zScoreNorm(std::vector<Template> &templates, Face::LinAlg::ZScoreNormalizationResult &normParams)
{
    for (unsigned int templateIndex = 0; templateIndex < templates.size(); templateIndex++)
        Face::LinAlg::Normalization::zScoreNormalization(templates[templateIndex].featureVector, normParams);
}

std::vector<Face::LinAlg::Vector> Template::getVectors(std::vector<Template> &templates)
{
    std::vector<Face::LinAlg::Vector> result;
    for (const auto &t : templates)
    {
        result.push_back(t.featureVector);
    }
    return result;
}

void Template::stats(std::vector<Template> &templates, const std::string &outPath)
{
    int n = templates.size();
    int m = templates[0].featureVector.rows;

    for (int featureIndex = 0; featureIndex < m; featureIndex++)
    {
        std::vector<double> values;
        for (int templateIndex = 0; templateIndex < n; templateIndex++)
        {
            values.push_back(templates[templateIndex].featureVector(featureIndex));
        }

        Face::LinAlg::Vector valuesVec(values);
        double min = valuesVec.minValue();
        double max = valuesVec.maxValue();

        std::vector<double> histogram(10);
        for (int i = 0; i < n; i++)
        {
            int bin = (int)((values[i]-min)/(max-min) * 10);
            if (bin == 10) bin = 9;
            histogram[bin] += 1;
        }
        for (int i = 0; i < 10; i++)
        {
            histogram[i] = histogram[i]/n;
        }

        std::ofstream out(outPath + "-" + std::to_string(featureIndex));

        double x = min;
        double step = (max-min)/10;
        for (int i = 0; i < 10; i++)
        {
            out << x << ' ' << histogram[i] << std::endl;
            x += step;
        }

        out.flush();
    }
}

std::vector<Template> Template::createTemplates(const std::vector<Face::LinAlg::Vector> &rawData, const std::vector<int> &IDs,
                                            const Face::Biometrics::FeatureExtractor &extractor)
{
    unsigned int n = rawData.size();
    if (n != IDs.size()) throw FACELIB_EXCEPTION("IDs and rawData count mismatch");

    std::vector<Template> result;
    for (unsigned int i = 0; i < n; i++)
    {
        Template t;
        t.subjectID = IDs[i];
        t.featureVector = extractor.extract(rawData[i]);

        result.push_back(t);
    }

    return result;
}

std::vector<Template> Template::clone(const std::vector<Template> &src)
{
    std::vector<Template> result;
    for (unsigned int i = 0; i < src.size(); i++)
    {
        Template t;
        t.subjectID = src[i].subjectID;
        t.featureVector = src[i].featureVector.clone();
        result.push_back(t);
    }
    return result;
}
