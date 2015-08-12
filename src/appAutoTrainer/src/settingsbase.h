#pragma once

#include <string>

#include "faceCommon/helpers/cmdlineargsparser.h"
#include "faceCommon/settings/settings.h"

namespace Face {
namespace AutoTrainer {

class SettingsBase
{
public:
    enum class AlignType { None, ICP, Landmark };
    AlignType alignType;

    int ICPiters;
    float smoothCoef;
    int smoothIters;

    std::string alignTypeToString(AlignType alignType)
    {
        switch (alignType) {
        case AlignType::ICP:
            return "ICP";
        case AlignType::Landmark:
            return "Landmark";
        default:
            return "None";
        }
    }

    SettingsBase() {}
    virtual ~SettingsBase() {}

    void printHelp()
    {
        std::cout << "  --landmarks landmarks.yml" << std::endl;
        std::cout << "  --meanFaceForAlign model.obj" << std::endl;
        std::cout << "  --preAlignTemplate template.yml" << std::endl;
        std::cout << "  --ICPiters 100" << std::endl;
        std::cout << "  --smoothCoef 0.01" << std::endl;
        std::cout << "  --smoothIters 10" << std::endl;
    }

    void printSettings()
    {
        Face::Settings &s = Face::Settings::instance();

        std::cout << "  --align " << alignTypeToString(alignType) << std::endl;
        if (alignType == AlignType::ICP)
        {
            std::cout << "  --meanFaceForAlign " << s.settingsMap[s.MeanFaceModelPathKey].convert<std::string>() << std::endl;
            std::cout << "  --preAlignTemplate " << s.settingsMap[s.PreAlignTemplatePathKey].convert<std::string>() << std::endl;
            std::cout << "  --ICPiters " << ICPiters << std::endl;
        }
        else if (alignType == AlignType::Landmark)
        {
            std::cout << "  --landmarks " << s.settingsMap[s.MeanFaceModelLandmarksPathKey].convert<std::string>() << std::endl;
        }

        std::cout << "  --smoothCoef " << smoothCoef << std::endl;
        std::cout << "  --smoothIters " << smoothIters << std::endl;
    }

protected:
    bool parseAlignType(Face::Helpers::CmdLineArgsParser &cmdLineParser)
    {
        bool ok;
        std::string alignTypeS = cmdLineParser.getParamValue("--align", ok); if (!ok) return false;
        if (alignTypeS.compare("none") == 0)
            alignType = AlignType::None;
        else if (alignTypeS.compare("icp") == 0)
            alignType = AlignType::ICP;
        else if (alignTypeS.compare("landmark") == 0)
            alignType = AlignType::Landmark;
        else
        {
            return false;
        }

        return true;
    }

    void parseIcpSettings(Face::Helpers::CmdLineArgsParser &cmdLineParser)
    {
        Face::Settings &s = Face::Settings::instance();
        bool ok;

        std::string meanFaceForAlign = cmdLineParser.getParamValue("--meanFaceForAlign", ok);
        if (ok) s.settingsMap[s.MeanFaceModelPathKey] = meanFaceForAlign;

        std::string preAlignTemplate = cmdLineParser.getParamValue("--preAlignTemplate", ok);
        if (ok) s.settingsMap[s.PreAlignTemplatePathKey] = preAlignTemplate;

        ICPiters = cmdLineParser.getParamValueInt("--ICPiters", ok);
        if (!ok) ICPiters = 100;
    }

    void parseLandmarkSettings(Face::Helpers::CmdLineArgsParser &cmdLineParser)
    {
        Face::Settings &s = Face::Settings::instance();
        bool ok;

        std::string landmarks = cmdLineParser.getParamValue("--landmarks", ok);
        if (ok) s.settingsMap[s.MeanFaceModelLandmarksPathKey] = landmarks;
    }

    void parseSmoothSettings(Face::Helpers::CmdLineArgsParser &cmdLineParser)
    {
        bool ok;

        smoothCoef = cmdLineParser.getParamValueFloat("--smoothCoef", ok);
        if (!ok) smoothCoef = 0.01;

        smoothIters = cmdLineParser.getParamValueInt("--smoothIters", ok);
        if (!ok) smoothIters = 10;
    }

    void parseSettings(Face::Helpers::CmdLineArgsParser &cmdLineParser)
    {
        parseIcpSettings(cmdLineParser);
        parseLandmarkSettings(cmdLineParser);
        parseSmoothSettings(cmdLineParser);
    }
};

}
}
