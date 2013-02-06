#ifndef ALIGN_H
#define ALIGN_H

#include <vector>

#include "common.h"

class Align
{
public:
    Data &target;
    std::vector<Data> sources;

    Align(Data &target, std::vector<Data> &sources);

    void runSIFT();
    void runBruteForce();
};

#endif // ALIGN_H
