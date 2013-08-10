#ifndef SCORELEVELFUSIONWRAPPER_H
#define SCORELEVELFUSIONWRAPPER_H

#include <QVector>

#include "scorelevefusion.h"

class ScoreLevelFusionWrapper
{
public:
    static QVector<int> trainClassifier(ScoreLevelFusionBase &classifier, QList<ScoreLevelFusionComponent> &components);
};

#endif // SCORELEVELFUSIONWRAPPER_H
