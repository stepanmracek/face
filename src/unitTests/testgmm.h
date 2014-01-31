#ifndef TESTGMM_H
#define TESTGMM_H

#include <opencv2/ml/ml.hpp>

#include "linalg/common.h"
#include "linalg/loader.h"
#include "biometrics/scorelevefusion.h"

class TestGMM
{
public:
    static void TestSerialization()
    {
        cv::EM em(2);

        double trainingData[] = { 501, 10, 255, 10, 501, 255, 10, 501};
        Matrix samples(4, 2, trainingData);
        em.train(samples);

        cv::FileStorage fs("gmm", cv::FileStorage::WRITE);
        em.write(fs);
        fs.release();

        cv::EM em2;
        cv::FileStorage fs2("gmm", cv::FileStorage::READ);
        em2.read(fs2["StatModel.EM"]);

        cv::FileStorage fs3("gmm2", cv::FileStorage::WRITE);
        em2.write(fs3);
        fs3.release();
    }

    static void Bug()
    {
        cv::Mat samples;
        cv::Mat testInput;

        cv::FileStorage dataStorage("data.yml", cv::FileStorage::READ);
        dataStorage["samples"] >> samples;
        dataStorage["testInput"] >> testInput;

        cv::EM gmm;
        gmm.train(samples);
        std::cout << gmm.predict(testInput)[0] << std::endl;

        cv::FileStorage storage("gmm", cv::FileStorage::WRITE);
        gmm.write(storage);
        storage.release();

        cv::FileStorage storage2("gmm", cv::FileStorage::READ);
        cv::EM gmm2;
        gmm2.read(storage2["StatModel.EM"]);
        std::cout << gmm2.predict(testInput)[0] << std::endl;
    }

    static void TestLearnScoreGMMFusion()
    {
        Face::Biometrics::ScoreGMMFusion fusion;

        QList<Face::Biometrics::Evaluation> allEvaluations;
        QList<Face::Biometrics::Evaluation> allEvaluations2;
        for (int modality = 0; modality <= 10; modality++)
        {
            QString modalityS = QString::number(modality);
            QVector<QString> files = Face::LinAlg::Loader::listFiles(modalityS, "*-*", Face::LinAlg::Loader::Filename);

            QVector<Face::Biometrics::Template> templates;
            foreach (const QString &f, files)
            {
                Face::LinAlg::Vector vec = Face::LinAlg::Vector::fromFile(modalityS + QDir::separator() + f);
                int id = f.split('-')[0].toInt();

                templates << Face::Biometrics::Template(id, vec);
            }

            Face::Biometrics::Evaluation eval(templates, Face::LinAlg::CorrelationMetric());
            qDebug() << modality << eval.eer;
            fusion.addComponent(eval);
            allEvaluations << eval;
            allEvaluations2 << eval;
        }

        fusion.learn();
        Face::Biometrics::Evaluation evaluation = fusion.evaluate(allEvaluations);
        evaluation.outputResults("gmm", 50);
        fusion.serialize("gmm");
        qDebug() << evaluation.eer;

        Face::Biometrics::ScoreGMMFusion fusion2("gmm");
        Face::Biometrics::Evaluation evaluation2 = fusion2.evaluate(allEvaluations2);
        qDebug() << evaluation2.eer;
        fusion2.serialize("gmm2");
    }
};

#endif // TESTGMM_H
