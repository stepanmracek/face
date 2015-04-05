#include "faceCommon/biometrics/biodataprocessing.h"

#include <Poco/StringTokenizer.h>
#include <Poco/NumberParser.h>

using namespace Face::Biometrics;

/**
 * @brief Divides templates into desired count of clusters. There will be exactly the same count of individual
 * subjects in each cluster, excluding the last one, where the remain subjects will be stored. The exact
 * amount of scans within each cluster may vary since the number of scans per one subject may vary.
 * *NOTE*: the method expects that the input templates are sorted (e.g. [sub1, sub1, sub1, sub2, sub2, sub3,...])
 * @param templates Input sorted templates
 * @param subjectsInOneCluster Desired count of subjects in each resulting cluster
 * @return Returns templates divided into clusters
 */
std::vector<std::vector<Template>> BioDataProcessing::divideTemplatesToClusters(const std::vector<Template> &templates, unsigned int subjectsInOneCluster)
{
    if (subjectsInOneCluster <= 1)
        throw FACELIB_EXCEPTION("there should be at least one subject in each cluster");

    std::set<int> currentClusterClasses;
    std::vector< std::vector<Template> > result;
    int currentClusterIndex = 0;

    //init
    std::vector<Template> ts;
    result.push_back(ts);

    // Iterate through the input templates
    for (const Template &t : templates)
    {
        // If the current subject is already stored in current cluster
        if (currentClusterClasses.count(t.subjectID) > 0)
        {
            // ok, add it
            result[currentClusterIndex].push_back(t);
        }
        else
        {
            // nope. We have to check count of subjects in current cluster
            if (currentClusterClasses.size() >= subjectsInOneCluster)
            {
                // Subjects count exceeded, we have to crate new cluster
                currentClusterClasses.clear();
                std::vector<Template> ts;
                result.push_back(ts);
                currentClusterIndex++;

                // We can add current subject to new  cluster
                currentClusterClasses.insert(t.subjectID);
                result[currentClusterIndex].push_back(t);
            }
            else
            {
                // We can add current subject to current cluster
                currentClusterClasses.insert(t.subjectID);
                result[currentClusterIndex].push_back(t);
            }
        }
    }

    return result;
}

/**
 * @brief Divides vectors and corresponding classes into desired count of clusters. There will be exactly the same count of
 * individual subjects in each cluster, excluding the last one, where the remain subjects will be stored. The exact
 * amount of scans within each cluster may vary since the number of scans per one subject may vary.
 * *NOTE #1*: the method expects that the input vectors are sorted (e.g. [sub1, sub1, sub1, sub2, sub2, sub3,...])
 * *NOTE #2*: the class membership have to correspond to vector data
 * @param vectors Input vectors
 * @param classMembership Input class membership
 * @param subjectsInOneCluster Desired count of clusters
 * @param resultVectors Resulting vectors divided to clusters
 * @param resultClasses Resulting classes divided to clusters
 */
void BioDataProcessing::divideVectorsToClusters(std::vector<Face::LinAlg::Vector> &vectors, std::vector<int> &classMembership, unsigned int subjectsInOneCluster,
                                                std::vector<std::vector<Face::LinAlg::Vector> > &resultVectors, std::vector<std::vector<int> > &resultClasses)
{
    unsigned int n = vectors.size();
    if (n != classMembership.size())
        throw FACELIB_EXCEPTION("classMembership and vectors count mismatch");
    if (subjectsInOneCluster <= 1)
        throw FACELIB_EXCEPTION("there should be at least one subject in each cluster");
    std::set<int> currentClusterClasses;
    int currentResultIndex = 0;

    //init
    std::vector<Face::LinAlg::Vector> vs;
    std::vector<int> cs;
    resultVectors.push_back(vs);
    resultClasses.push_back(cs);

    // iterate through the input data
    for (unsigned int i = 0; i < n; i++)
    {
        Face::LinAlg::Vector &v = vectors[i];
        int c = classMembership[i];
        // If the current subject is already stored in current cluster
        if (currentClusterClasses.count(c) > 0)
        {
            // ok, add it
            resultVectors[currentResultIndex].push_back(v);
            resultClasses[currentResultIndex].push_back(c);
        }
        else
        {
            // nope. We have to check count of subjects in current cluster
            if (currentClusterClasses.size() >= subjectsInOneCluster)
            {
                // Subjects count exceeded, we have to crate new cluster
                currentClusterClasses.clear();
                std::vector<Face::LinAlg::Vector> vs;
                std::vector<int> cs;
                resultVectors.push_back(vs);
                resultClasses.push_back(cs);
                currentResultIndex++;

                // We can add current subject to current cluster
                currentClusterClasses.insert(c);
                resultVectors[currentResultIndex].push_back(v);
                resultClasses[currentResultIndex].push_back(c);
            }
            else
            {
                // We can add current subject to current cluster
                currentClusterClasses.insert(c);
                resultVectors[currentResultIndex].push_back(v);
                resultClasses[currentResultIndex].push_back(c);
            }
        }
    }
}

std::vector<int> BioDataProcessing::getIds(const std::vector<std::string> &fileNames, const std::string &classSeparator)
{
    std::vector<int> result;
    for (const std::string &f : fileNames)
    {
        result.push_back(Poco::NumberParser::parse(Poco::StringTokenizer(f, classSeparator)[0]));
    }
    return result;
}
