#ifndef SCORELEVELFUSIONWRAPPER_H
#define SCORELEVELFUSIONWRAPPER_H

#include "scorelevefusion.h"

namespace Face {
namespace Biometrics {

class ScoreLevelFusionWrapper
{
public:
    class Result
    {
    public:
        std::vector<int> selectedComponents;
        ScoreLevelFusionBase::Ptr fusion;
    };

    static Result trainClassifier(const std::string &fusionName, const std::vector<Evaluation> &components, bool debugOutput);
};

}
}

#endif // SCORELEVELFUSIONWRAPPER_H
