#ifndef FRGCUTILS_H
#define FRGCUTILS_H

#include "faceCommon/linalg/common.h"

namespace Face {
namespace Helpers {

class FRGCUtils
{
public:
    class Spring2004
    {
    public:
        static int part1Size() { return 416; }
        static int part2Size() { return 451; }
        static int part3Size() { return 414; }
        static int part4Size() { return 417; }
        static int part5Size() { return 380; }

        static int partSize(int part)
        {
            switch (part)
            {
            case 1:
                return part1Size();
            case 2:
                return part2Size();
            case 3:
                return part3Size();
            case 4:
                return part4Size();
            case 5:
                return part5Size();
            default:
                throw FACELIB_EXCEPTION("FRGC-Spring2004 has only 5 parts.");
            }
        }

        static int part1Start() { return 0; }
        static int part2Start() { return part1Start() + part1Size(); }
        static int part3Start() { return part2Start() + part2Size(); }
        static int part4Start() { return part3Start() + part3Size(); }
        static int part5Start() { return part4Start() + part4Size(); }

        static int partStart(int part)
        {
            switch (part)
            {
            case 1:
                return part1Start();
            case 2:
                return part2Start();
            case 3:
                return part3Start();
            case 4:
                return part4Start();
            case 5:
                return part5Start();
            default:
                throw FACELIB_EXCEPTION("FRGC-Spring2004 has only 5 parts.");
            }
        }
    };
};

}
}

#endif // FRGCUTILS_H
