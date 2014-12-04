#include "faceCommon/biometrics/featurelevelfusion.h"

#include "faceCommon/linalg/vector.h"

using namespace Face::Biometrics;

FeatureVectorFusionBase & FeatureVectorFusionBase::addComponent(std::vector<Face::LinAlg::Vector> &trainRawData,
        std::vector<int> &trainClasses,
        Face::Biometrics::FeatureExtractor &featureExtractor,
        Face::LinAlg::Metrics &metrics)
{
    this->extractors.push_back(&featureExtractor);
    this->metrics.push_back(&metrics);
    this->trainClasses.push_back(&trainClasses);
    this->trainRawData.push_back(&trainRawData);

	return (*this);
}

void FeatureVectorFusionBase::learn()
{
	learnImplementation();
	learned = true;
}

std::vector<Face::LinAlg::Vector> FeatureVectorFusionBase::batchFuse(std::vector<std::vector<Face::LinAlg::Vector> > inputMatricies)
{
    if (!isLearned()) throw FACELIB_EXCEPTION("fusion is not trained");
    int unitsCount = inputMatricies.size();
    if (unitsCount == 0) throw FACELIB_EXCEPTION("fusion has no input units");

    int matriciesPerUnitCount = inputMatricies[0].size();

    std::vector<Face::LinAlg::Vector> resultVector;
	for (int i = 0; i < matriciesPerUnitCount; i++)
	{
        std::vector<Face::LinAlg::Vector> input;
		for (int j = 0; j < unitsCount; j++)
		{
            input.push_back(inputMatricies[j][i]);
		}
        Face::LinAlg::Vector result = fuse(input);
        resultVector.push_back(result);
	}
    return std::move(resultVector);
}

// --- Concatenation ---

Face::LinAlg::Vector FeatureVectorFusionConcatenation::fuse(std::vector<Face::LinAlg::Vector> &inputMatricies)
{
    int n = inputMatricies.size();
	std::vector<double> vec;
	for (int i = 0; i < n; i++)
	{
		int r = inputMatricies[i].rows;
		for (int j = 0; j < r; j++)
		{
            vec.push_back(inputMatricies[i](j));
		}
	}

    Face::LinAlg::Vector result(vec);
	return result;
}
