#include "faceCommon/helpers/cmdlineargsparser.h"

#include <Poco/NumberParser.h>
#include <algorithm>

using namespace Face::Helpers;

CmdLineArgsParser::CmdLineArgsParser(int argc, char *argv[])
{
    params = createParamsList(argc, argv);
}

std::vector<std::string> CmdLineArgsParser::createParamsList(int argc, char *argv[])
{
    std::vector<std::string> result;
    for (int i = 0; i < argc; i++)
        result.push_back(argv[i]);
    return result;
}

std::string CmdLineArgsParser::getParamValue(const std::string &key, bool &ok)
{
    unsigned int i = std::find(params.begin(), params.end(), key) - params.begin();
    if (i < params.size() - 1)
    {
        ok = true;
        return params[i+1];
    }
    else
    {
        ok = false;
        return std::string();
    }
}

int CmdLineArgsParser::getParamValueInt(const std::string &key, bool &ok)
{
    std::string s = getParamValue(key, ok);
    if (!ok) return 0;

    int val;
    ok = Poco::NumberParser::tryParse(s, val);
    return val;
}

double CmdLineArgsParser::getParamValueFloat(const std::string &key, bool &ok)
{
    std::string s = getParamValue(key, ok);
    if (!ok) return 0;

    double val;
    ok = Poco::NumberParser::tryParseFloat(s, val);
    return val;
}

bool CmdLineArgsParser::hasParam(const std::string &param)
{
    return std::find(params.begin(), params.end(), param) != params.end();
}


