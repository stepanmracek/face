#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <string>

#include "faceCommon/biometrics/evaluation.h"

namespace Face {
namespace AutoTrainer {

class Evaluator
{
public:
    class Settings
    {
    public:
        Settings() {}
        Settings(int argc, char *argv[], bool &ok);

        std::string extractor;
        std::string meanFaceForAlign;
        std::string preAlignTemplate;
        std::string evalDir;
        int frgcSamples;
        int ICPiters;
        float smoothCoef;
        int smoothIters;

        static void printHelp();
        void printSettings();
    };

    static Biometrics::Evaluation evaluate(const Settings &settings);
};

}
}

#endif // EVALUATOR_H
