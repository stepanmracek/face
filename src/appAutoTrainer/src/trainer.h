#ifndef TRAINER_H
#define TRAINER_H

#include "faceCommon/biometrics/multibiomertricsautotuner.h"
#include "faceCommon/facedata/facealigner.h"
#include "settingsbase.h"

namespace Face {
namespace AutoTrainer {

class Trainer
{
public:
    class Settings : public SettingsBase
    {
    public:
        Settings() {}
        Settings(int argc, char *argv[], bool &ok);

        std::string unitsFile;
        std::string fusionName;
        std::string frgcDir;
        std::string trainDir;
        bool noWrapper;
        int frgcSamples;

        void printHelp();
        void printSettings();
    };

    static Face::Biometrics::MultiExtractor::Ptr train(const Settings &settings);
};

}
}

#endif // TRAINER_H
