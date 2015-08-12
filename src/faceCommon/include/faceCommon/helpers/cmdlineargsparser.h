#ifndef CMDLINEARGSPARSER_H
#define CMDLINEARGSPARSER_H

#include <string>
#include <vector>

#include "faceCommon/faceCommon.h"

namespace Face {
namespace Helpers {

class FACECOMMON_EXPORTS CmdLineArgsParser
{
    std::vector<std::string> params;

    std::vector<std::string> createParamsList(int argc, char *argv[]);

public:
    CmdLineArgsParser(int argc, char *argv[]);

    std::string getParamValue(const std::string &key, bool &ok);
    int getParamValueInt(const std::string &key, bool &ok);
    double getParamValueFloat(const std::string &key, bool &ok);
    bool hasParam(const std::string &param);
};

}
}

#endif // CMDLINEARGSPARSER_H
