#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <string>

namespace Face
{
namespace AutoTrainer
{

class Processor
{
public:
    class Settings
    {
    public:
        Settings() {}
        Settings(int argc, char *argv[], bool &ok);

        std::string meanFaceForAlign;
        std::string preAlignTemplate;
        std::string inputDir;
        std::string outputDir;
        int ICPiters;
        float smoothCoef;
        int smoothIters;

        static void printHelp();
        void printSettings();
    };

    static void process(const Settings &settings);
};

}
}

#endif // PROCESSOR_H
