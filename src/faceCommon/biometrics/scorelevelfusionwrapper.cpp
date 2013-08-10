#include "scorelevelfusionwrapper.h"

QVector<int> ScoreLevelFusionWrapper::trainClassifier(ScoreLevelFusionBase &classifier,
                                                      QList<ScoreLevelFusionComponent> &components)
{
    QVector<int> selectedComponents;
    int n = components.count();
    qDebug() << "Training score-level fusion classifier:" << n << "candidate components";

    // select the best component as the base for the classifier
    double bestEER = 1;
    int bestIndex = -1;
    for (int i = 0; i < n; i++)
    {
        double eer = Evaluation(components[i].trainRawData, components[i].trainClasses,
                                *components[i].featureExtractor, *components[i].metrics).eer;
        qDebug() << "Component" << i << ", EER:" << eer;
        if (eer < bestEER)
        {
            bestEER = eer;
            bestIndex = i;
        }
    }
    assert(bestIndex != -1);
    selectedComponents << bestIndex;
    classifier.addComponent(components[bestIndex]);

    qDebug() << "Selected" << bestIndex << "with EER" << bestEER << "as the base for the classifier";

    // iteratively add the best remaining components until there is some improvement
    bool improvement = true;
    while (improvement)
    {
        improvement = false;
        bestIndex = -1;
        for (int i = 0; i < n; i++)
        {
            // if the component was already selected skip it
            if (selectedComponents.contains(i)) continue;

            // lear the classifier
            classifier.addComponent(components[i]);
            classifier.learn();

            // create the input
            QList<QVector<Vector> > rawVectors;
            foreach(int component, selectedComponents)
            {
                rawVectors << components[component].trainRawData;
            }
            rawVectors << components[i].trainRawData;

            // evaluate
            double eer = classifier.evaluate(rawVectors, components[i].trainClasses).eer;
            qDebug() << "  trying to add classifier" << i << ", fusion EER:" << eer;
            if (eer < bestEER)
            {
                bestEER = eer;
                improvement = true;
                bestIndex = i;
            }
            classifier.popComponent();
        }

        if (improvement)
        {
            classifier.addComponent(components[bestIndex]);
            classifier.learn();
            selectedComponents << bestIndex;
            qDebug() << "added classifier" << bestIndex << ", fusion EER:" << bestEER;
        }
    }

    classifier.learn();
    qDebug() << "final result" << selectedComponents << "EER:" << bestEER;
    return selectedComponents;
}
