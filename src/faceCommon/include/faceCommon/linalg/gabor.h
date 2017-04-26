#pragma once

#include "common.h"
#include "kernelgenerator.h"
#include "filterbank.h"

namespace Face {
namespace LinAlg {

class FACECOMMON_EXPORTS Gabor : public FilterBank
{
public:
    Gabor();

    static void createWavelet(Matrix &real, Matrix &imag, int frequency, int orientation);
private:
    static void gaborFunc(int x, int y, double omega, double theta, double sigma, double &real, double &imag);
};

}
}
