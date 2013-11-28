#ifndef REALTIMECLASSIFIER_H
#define REALTIMECLASSIFIER_H

#include <QString>
#include <QHash>

#include "facetemplate.h"
#include "facelib/facealigner.h"

class RealTimeClassifier
{
public:
    RealTimeClassifier(const FaceClassifier &classifier, double threshold, FaceClassifier::ComparisonType comparisonType,
                       const QHash<int, FaceTemplate*> &database, const QString &pathToAlignReference);

    const FaceClassifier &classifier;
    double threshold;
    FaceClassifier::ComparisonType comparisonType;
    const QHash<int, FaceTemplate*> &database;
    Mesh mean;
    FaceAligner aligner;
    double minDistance;
    int minDistanceId;
    bool comparing;

    void compare(const Mesh *in);
};

#endif // REALTIMECLASSIFIER_H
