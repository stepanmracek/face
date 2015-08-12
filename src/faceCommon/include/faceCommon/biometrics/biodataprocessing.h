#ifndef BIODATAPROCESSING_H
#define BIODATAPROCESSING_H

#include "template.h"
#include "faceCommon/linalg/common.h"

namespace Face {
namespace Biometrics {

class FACECOMMON_EXPORTS BioDataProcessing
{
public:
    static std::vector<std::vector<Template>> divideTemplatesToClusters(const std::vector<Template> &templates, unsigned int subjectsInOneCluster);

    static void divideVectorsToClusters(std::vector<Face::LinAlg::Vector> &vectors, std::vector<int> &classMembership, unsigned int subjectsInOneCluster,
                                        std::vector<std::vector<Face::LinAlg::Vector> > &resultVectors, std::vector<std::vector<int> > &resultClasses);

    static std::vector<int> getIds(const std::vector<std::string> &fileNames, const std::string &classSeparator);

    template <class T>
    /**
     * @brief Divides input data into desired count of clusters
     * @param vectors Input vectors
     * @param classMembership Input classes
     * @param numberOfClusters Desired count of resulting clusters
     * @param resultVectors
     * @param resultClasses
     */
    static void divideToNClusters(const std::vector<T> &vectors, const std::vector<int> &classMembership, int numberOfClusters,
                                  std::vector<std::vector<T> > &resultVectors, std::vector<std::vector<int> > &resultClasses)
    {
        int n = vectors.size();
        if (n != classMembership.size())
            throw FACELIB_EXCEPTION("classMembership and vectors count mismatch");
        if (numberOfClusters <= 1)
            throw FACELIB_EXCEPTION("there should be at least two clusters");

        // Map class id to its corresponding vectors
        std::map<int, std::vector<T> > classToVectors;
        for (int i = 0; i < n; i++)
            classToVectors[classMembership[i]].push_back(vectors[i]);

        // initialize resulting output
        int countPerCluster = classToVectors.size()/numberOfClusters;
        int currentCluster = 0;
        for (int i = 0; i < numberOfClusters; i++)
        {
            std::vector<T> data;
            std::vector<int> classIds;
            resultVectors.push_back(data);
            resultClasses.push_back(classIds);
        }

        // for each class
        for (const auto &c2vPair : classToVectors)
        {
            // unique classes in current cluster
            std::set<int> currentClasses(resultClasses[currentCluster].begin(), resultClasses[currentCluster].end());

            // need to increment cluster index?
            int currentCount = currentClasses.size();
            if (currentCluster != (numberOfClusters-1) && currentCount >= countPerCluster)
                currentCluster++;

            // add all vectors that belongs to the class to the result
            int c = c2vPair.first;
            const std::vector<T> & curClassVectors = c2vPair.second;
            for (const T &v : curClassVectors)
            {
                resultClasses[currentCluster].push_back(c);
                resultVectors[currentCluster].push_back(v);
            }
        }
    }
};

}
}

#endif // BIODATAPROCESSING_H
