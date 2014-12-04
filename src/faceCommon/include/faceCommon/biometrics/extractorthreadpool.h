#ifndef EXTRACTORTHREADPOOL_H
#define EXTRACTORTHREADPOOL_H

#include <Poco/ThreadPool.h>

#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/biometrics/multitemplate.h"

namespace Face
{
namespace Biometrics
{

class MultiExtractor;

class ExtractorThreadPool : public Poco::ThreadPool
{
    class Thread : public Poco::Runnable
    {
        const MultiExtractor::ImageData *data;
        MultiTemplate *t;
        const MultiExtractor *extractor;
        int startIndex;
        int endIndex;

    public:
        void setUp(const MultiExtractor::ImageData *data, MultiTemplate *t, const MultiExtractor *extractor, int startIndex, int endIndex);
        void run();
    };

    std::vector<Thread> threads;

public:
    ExtractorThreadPool();

    void extract(const MultiExtractor::ImageData *data, MultiTemplate *t, const MultiExtractor *extractor);
};

}
}

#endif // EXTRACTORTHREADPOOL_H
