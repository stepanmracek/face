#include "faceCommon/biometrics/extractorthreadpool.h"

#include <Poco/Environment.h>

using namespace Face::Biometrics;

void ExtractorThreadPool::Thread::setUp(const MultiExtractor::ImageData *data, MultiTemplate *t,
                                        const MultiExtractor *extractor, int startIndex, int endIndex)
{
    this->data = data;
    this->t = t;
    this->extractor = extractor;
    this->startIndex = startIndex;
    this->endIndex = endIndex;
}

void ExtractorThreadPool::Thread::run()
{
    for (int index = startIndex; index < endIndex; index++)
        t->featureVectors[index] = extractor->units[index]->extract(*data);
}

ExtractorThreadPool::ExtractorThreadPool()
{
    threads = std::vector<ExtractorThreadPool::Thread>(Poco::Environment::processorCount());
}

void ExtractorThreadPool::extract(const MultiExtractor::ImageData *data, MultiTemplate *t, const MultiExtractor *extractor)
{
    for (unsigned int i = 0; i < threads.size(); i++)
    {
        int startIndex = i * extractor->units.size() / threads.size();
        int endIndex = (i+1) * extractor->units.size() / threads.size();

        //std::cout << i << " " << startIndex << " " << endIndex << " " << extractor->units.count() << std::endl;

        threads[i].setUp(data, t, extractor, startIndex, endIndex);
        start(threads[i], "extractor-"+std::to_string(i));
    }
    joinAll();
}
