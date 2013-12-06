#include "realtimeclassifier.h"

RealTimeClassifier::RealTimeClassifier(const FaceClassifier &classifier, double threshold, FaceClassifier::ComparisonType comparisonType,
                                       const QHash<int, Face3DTemplate *> &database, const QString &pathToAlignReference) :
    classifier(classifier),
    threshold(threshold),
    comparisonType(comparisonType),
    database(database),
    mean(Mesh::fromOBJ(pathToAlignReference)),
    aligner(mean),
    minDistance(1e300),
    minDistanceId(-1),
    comparing(false)
{

}

void RealTimeClassifier::compare(const Mesh *in)
{
    comparing = true;

    if (!in)
    {
        comparing = false;
        return;
    }

    Mesh probe(*in);
    probe.calculateTriangles();
    aligner.icpAlign(probe, 10, FaceAligner::NoseTipDetection);

    Face3DTemplate probeTemplate(0, probe, classifier);
    QMap<int, double> result = classifier.identify(database, &probeTemplate, comparisonType);

    foreach (int id, result.keys())
    {
        double d = result[id];
        if (d < minDistance && d <= threshold)
        {
            minDistanceId = id;
            minDistance = d;
        }
    }

    comparing = false;
}
