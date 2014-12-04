#ifndef TEMPLATEMATCHINGTRAINER_H
#define TEMPLATEMATCHINGTRAINER_H

#include <vector>
#include <string>

namespace Face
{
namespace FaceData
{

class Mesh;

class TemplateMatchingTrainer
{
private:
    static void evaluate(const std::vector<Mesh> &alignedMeshes, const std::vector<int> &ids, const std::string &pcaDepthmapPath);

public:  
    static void evaluateBaseline(const std::string &meanForAlign, const std::string &dataPath, const std::string &pcaDepthmapPath);
    static void train(const std::string &meanForAlign, const std::string &dataPath, const std::string &pcaDepthmapPath);
};

}
}

#endif // TEMPLATEMATCHINGTRAINER_H
