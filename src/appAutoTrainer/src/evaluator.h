#ifndef EVALUATOR_H
#define EVALUATOR_H

#include <string>

#include "faceCommon/biometrics/evaluation.h"
#include "settingsbase.h"

namespace Face {
namespace AutoTrainer {

class Evaluator
{
public:
    class Settings : public SettingsBase
    {
    public:
        Settings() {}
        Settings(int argc, char *argv[], bool &ok);

        std::string extractor;
        std::string evalDir;

        void printHelp();
        void printSettings();
    };

    static Biometrics::Evaluation evaluate(const Settings &settings);
};

}
}

#endif // EVALUATOR_H
