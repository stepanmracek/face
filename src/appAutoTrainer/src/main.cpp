#include "trainer.h"
#include "evaluator.h"
#include "templateextractor.h"
#include "processor.h"

using namespace Face::AutoTrainer;

int main(int argc, char *argv[])
{
    if (argc >= 3 && std::string(argv[1]) == "--trainUnits")
    {
        bool ok;
        Trainer::Settings settings(argc, argv, ok);
        if (!ok)
        {
            settings.printHelp();
            return 0;
        }
        settings.printSettings();

        Face::Biometrics::MultiExtractor::Ptr extractor = Trainer::train(settings);
        extractor->serialize(argv[2]);
    }
    else if (argc >= 3 && std::string(argv[1]) == "--evaluate")
    {
        bool ok;
        Evaluator::Settings settings(argc, argv, ok);
        if (!ok)
        {
            settings.printHelp();
            return 0;
        }
        settings.printSettings();

        Face::Biometrics::Evaluation evaluation = Evaluator::evaluate(settings);
        evaluation.printStats();
        evaluation.outputResults(argv[2], 50);
    }
    /*else if (argc >= 2 && std::string(argv[1]) == "--extract")
    {
        bool ok;
        TemplateExtractor::Settings settings(argc, argv, ok);
        if (!ok)
        {
            TemplateExtractor::Settings::printHelp();
            return 0;
        }
        settings.printSettings();

        TemplateExtractor::extract(settings);
    }
    else if (argc >= 2 && std::string(argv[1]) == "--process")
    {
        bool ok;
        Processor::Settings settings(argc, argv, ok);
        if (!ok)
        {
            Processor::Settings::printHelp();
            return 0;
        }
        settings.printSettings();

        Processor::process(settings);
    }*/
    else
    {
        std::cout << "usage: " << argv[0] << " mode params" << std::endl;
        std::cout << "modes:" << std::endl;
        std::cout << "  --trainUnits resultClassifierPath" << std::endl;
        std::cout << "  --evaluate   serializedEvaluationPrefix" << std::endl;
        //std::cout << "  --extract" << std::endl;
        //std::cout << "  --process" << std::endl;
    }

    return 0;
}
