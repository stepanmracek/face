#include "evaluatelbp.h"

#include "faceCommon/linalg/loader.h"
#include "faceCommon/biometrics/biodataprocessing.h"
#include "faceCommon/biometrics/multiextractor.h"

typedef Face::Biometrics::MultiExtractor::Unit Unit;

/*std::vector<Face::LinAlg::Vector> processFilters(Unit::Ptr unit, std::vector<Matrix> &inputData)
{
    std::vector<Face::LinAlg::Vector> result;
    foreach (const Matrix &m, inputData)
    {
        result << Face::LinAlg::MatrixConverter::matrixToColumnVector(
                      Face::LinAlg::ImageFilter::batchProcess(m, unit->imageFilters));
    }
    return result;
}*/

/*std::vector<Face::LinAlg::Vector> process(Unit::Ptr unit, std::vector<Matrix> &inputData)
{
    std::vector<Face::LinAlg::Vector> tmp = processFilters(unit, inputData);
    std::vector<Face::LinAlg::Vector> result;
    foreach (const Face::LinAlg::Vector &v, tmp)
    {
        result << unit->featureExtractor->extract(v);
    }
    return result;
}*/

/*void EvaluateLBP::evaluate()
{
    QString imageType = "index";
    std::vector<Matrix> allMatrices;
    std::vector<int> allClasses;
    Face::LinAlg::Loader::loadMatrices(frgcPath() + imageType, allMatrices, allClasses, "d", "*.gz");
    qDebug() << "loaded" << allMatrices.count() << "images";

    QList<std::vector<Matrix> > matricesInClusters;
    QList<std::vector<int> > classesInClusters;
    Face::Biometrics::BioDataProcessing::divideToNClusters(allMatrices, allClasses, 5, matricesInClusters, classesInClusters);

    QStringList units;
    units << "index zpca scale-0.5 correlation"
          << "index pass wld-10-9 cityblock"
          << "index zpca wld-10-9 correlation"
          << "index pass wld-5-4 cityblock"
          << "index zpca wld-5-4 correlation";

    foreach (const QString &unitLine, units)
    {
        Unit::Ptr unit = Unit::parse(unitLine);
        unit->featureExtractor->train(classesInClusters[0], processFilters(unit, matricesInClusters[0]));

        Face::Biometrics::Evaluation eval(processFilters(unit, matricesInClusters[1]), classesInClusters[1],
                                          *unit->featureExtractor, *unit->metrics);
        qDebug() << unitLine << eval.eer;
    }
}*/
