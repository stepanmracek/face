#ifndef TEMPLATEEXTRACTOR_H
#define TEMPLATEEXTRACTOR_H

#include <string>

namespace Face {
namespace AutoTrainer {

class TemplateExtractor
{
public:
    class Settings
    {
    public:
        Settings() {}
        Settings(int argc, char *argv[], bool &ok);

        std::string extractorPath;
        std::string inputPath;
        std::string resultPath;
        std::string meanFaceForAlign;
        std::string preAlignTemplate;
        int ICPiters;
        float smoothCoef;
        int smoothIters;
        int templateVersion;

        static void printHelp();
        void printSettings();
    };

    static void extract(const Settings &settings);
};

}
}

#endif // TEMPLATEEXTRACTOR_H
