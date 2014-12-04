#ifndef FEATUREVECTORFUSION_H_
#define FEATUREVECTORFUSION_H_

#include "faceCommon/linalg/common.h"
#include "faceCommon/linalg/vector.h"
#include "faceCommon/linalg/lda.h"
#include "faceCommon/linalg/metrics.h"
#include "featureextractor.h"
#include "template.h"

namespace Face {
namespace Biometrics {

class FeatureVectorFusionBase
{
private:
    bool learned;

protected:
    std::vector<std::vector<Face::LinAlg::Vector> *> trainRawData;
	std::vector<std::vector<int> *> trainClasses;
    std::vector<Face::Biometrics::FeatureExtractor *> extractors;
    std::vector<Face::LinAlg::Metrics *> metrics;

	virtual void learnImplementation() = 0;

public:
	FeatureVectorFusionBase() { learned = false; }

	FeatureVectorFusionBase & addComponent(
                std::vector<Face::LinAlg::Vector> &trainRawData,
				std::vector<int> &trainClasses,
                Face::Biometrics::FeatureExtractor &featureExtractor,
                Face::LinAlg::Metrics &metrics);

    std::vector<Face::LinAlg::Vector> batchFuse(std::vector<std::vector<Face::LinAlg::Vector> > inputMatricies);

	void learn();

    virtual Face::LinAlg::Vector fuse(std::vector<Face::LinAlg::Vector> &inputMatricies) = 0;

	virtual ~FeatureVectorFusionBase() {}

    bool isLearned() { return learned; }
};

class FeatureVectorFusionConcatenation : public FeatureVectorFusionBase
{
public:
    Face::LinAlg::Vector fuse(std::vector<Face::LinAlg::Vector> &inputMatricies);

	// no nead to learn for simple concatenation
	void learnImplementation() {}

	virtual ~FeatureVectorFusionConcatenation() {}
};

}
}

#endif /* FEATUREVECTORFUSION_H_ */
