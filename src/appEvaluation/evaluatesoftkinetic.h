#ifndef EVALUATESOFTKINETIC_H
#define EVALUATESOFTKINETIC_H

#include "facelib/facealigner.h"
#include "biometrics/facetemplate.h"
#include "facelib/surfaceprocessor.h"
#include "biometrics/multibiomertricsautotuner.h"

class EvaluateSoftKinetic
{
public:
    class MeshProcessor
    {
    public:
        virtual void process(Mesh &input) = 0;
    };

    class PassProcess : public MeshProcessor
    {
    public:
        virtual void process(Mesh &) {}
    };

    class Smooth : public MeshProcessor
    {
        int iterations;
        double alpha;

    public:
        Smooth(int iterations, double alpha) : iterations(iterations), alpha(alpha) { }

        void process(Mesh &input)
        {
            SurfaceProcessor::smooth(input, alpha, iterations);
        }
    };

    class ZSmooth : public MeshProcessor
    {
        int iterations;
        double alpha;

    public:
        ZSmooth(int iterations, double alpha) : iterations(iterations), alpha(alpha) { }

        void process(Mesh &input)
        {
            SurfaceProcessor::zsmooth(input, alpha, iterations);
        }
    };

    class AnisoSmooth : public MeshProcessor
    {
        SurfaceProcessor::AnisotropicDiffusionType type;
        double edgeThresh;
        int steps;
        double dt;

    public:
        AnisoSmooth(SurfaceProcessor::AnisotropicDiffusionType type, double edgeThresh, int steps, double dt) :
            type(type), edgeThresh(edgeThresh), steps(steps), dt(dt) { }

        void process(Mesh &input)
        {
            SurfaceProcessor::anisotropicDiffusionSmooth(input, type, edgeThresh, steps, dt);
        }
    };

    static void evaluateSmoothing()
    {
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
        FaceClassifier faceClassifier("../../test/frgc/classifiers/");
        QVector<Mesh> meshes;
        QVector<int> ids;

        QVector<QString> binFiles = Loader::listFiles("../../test/softKinetic/02/", "*.binz", Loader::AbsoluteFull);
        foreach(const QString &path, binFiles)
        {
            ids << QFileInfo(path).baseName().split("-")[0].toInt();
            meshes <<  Mesh::fromBINZ(path);
            aligner.icpAlign(meshes.last(), 10, FaceAligner::NoseTipDetection);
        }

        QVector<MeshProcessor *> processors;
        //processors << new PassProcess();
        processors << new ZSmooth(5, 0.5); //!!!
        //processors << new ZSmooth(10, 0.5);
        //processors << new ZSmooth(5, 1);
        //processors << new ZSmooth(10, 1);
        //processors << new AnisoSmooth(SurfaceProcessor::PeronaMalic, 5, 8, 0.04); //!!!
        //processors << new AnisoSmooth(SurfaceProcessor::PeronaMalic, 10, 16, 0.02);
        //processors << new AnisoSmooth(SurfaceProcessor::Linear, 0.5, 12, 0.04);

        foreach(MeshProcessor *p, processors)
        {
            QVector<Face3DTemplate *> templates;

            for (int i = 0; i < meshes.count(); i++)
            {
                Mesh in(meshes[i]);
                p->process(in);
                templates << new Face3DTemplate(ids[i], in, faceClassifier);
            }

            FaceClassifier c = faceClassifier.relearnFinalFusion(templates, false);

            qDeleteAll(templates);
            templates.clear();

            for (int i = 0; i < meshes.count(); i++)
            {
                Mesh in(meshes[i]);
                p->process(in);
                templates << new Face3DTemplate(ids[i], in, c);
            }
            Evaluation e = c.evaluate(templates);
            qDebug() << e.eer << e.fnmrAtFmr(0.01) << e.fnmrAtFmr(0.001);
            qDeleteAll(templates);

            c.serialize("test");
        }
    }

    static int checkAligning(int argc, char *argv[])
    {
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
        QString dir("../../test/softKinetic/03/DS32528233700098_radim/");

        QVector<QString> fileNames = Loader::listFiles(dir, "*.binz", Loader::Filename);
        foreach(const QString &fileName, fileNames) {
            Mesh m = Mesh::fromBINZ(dir + fileName);
            SurfaceProcessor::mdenoising(m, 0.02f, 10, 10);
            aligner.icpAlign(m, 100, FaceAligner::TemplateMatching);
            MapConverter mc;
            Map texture = SurfaceProcessor::depthmap(m, mc, cv::Point(-100,-100), cv::Point(100,100), 1.0, SurfaceProcessor::Texture_I);
            Matrix mat = texture.toMatrix();
            cv::circle(mat, cv::Point(100,100), 2, 1.0);
            cv::circle(mat, cv::Point(100,100), 1, 0.0);
            cv::imshow("face", mat);
            cv::waitKey();
        }
        return 0;
    }

    static void loadBinZMeshes(const QString &dir, const FaceAligner &aligner, QVector<int> &ids, QVector<Mesh> &meshes, int icpIterations, int smoothIterations, float smoothCoef)
    {
        QVector<QString> fileNames = Loader::listFiles(dir, "*.binz", Loader::Filename);
        foreach(const QString &fileName, fileNames)
        {
            Mesh m = Mesh::fromBINZ(dir + fileName);
            SurfaceProcessor::mdenoising(m, smoothCoef, smoothIterations, smoothIterations); // 0.02f, 10, 10);
            aligner.icpAlign(m, icpIterations, FaceAligner::TemplateMatching); // 20

            ids << fileName.split('-')[0].toInt();
            meshes << m;
        }
    }

    static Evaluation evaluateMultiExtractor(const MultiExtractor &extractor, const FaceAligner &aligner, const QString &evalPath, int icpIterations, int smoothIterations, float smoothCoef)
    {
        QVector<int> ids;
        QVector<Mesh> meshes;
        loadBinZMeshes(evalPath, aligner, ids, meshes, icpIterations, smoothIterations, smoothCoef);
        int n = ids.count();
        QVector<MultiTemplate> templates;
        for (int i = 0; i < n; i++)
        {
            templates << extractor.extract(meshes[i], 1, ids[i]);
        }

        return extractor.evaluate(templates);
    }

    static void createMultiExtractor(int argc, char *argv[])
    {
        if (argc != 13)
        {
            qDebug() << "usage:" << argv[0] << "meanForAlign.obj unitsFile FRGCdir targetSoftKineticDir evaluationSoftKineticDir";
            qDebug() << "   trainICPiters testICPiters trainSmoothCoef testSmoothCoef trainSmoothIters testSmoothIters outputDir";
            return;
        }

        QString meanFaceForAlign = argv[1];
        QString unitsFile = argv[2];
        QString frgcDirectory = argv[3];
        QString targetSoftKineticDir = argv[4];
        QString evaluationSoftKineticDir = argv[5];
        int trainICPiters = QString(argv[6]).toInt();
        int testICPiters = QString(argv[7]).toInt();
        float trainSmoothCoef = QString(argv[8]).toFloat();
        float testSmoothCoef = QString(argv[9]).toFloat();
        int trainSmoothIters = QString(argv[10]).toInt();
        int testSmoothIters = QString(argv[11]).toInt();
        QString outputDir = argv[12];

        FaceAligner aligner(Mesh::fromOBJ(meanFaceForAlign, false));
        QVector<int> ids;
        QVector<Mesh> meshes;
        loadBinZMeshes(targetSoftKineticDir, aligner, ids, meshes, trainICPiters, trainSmoothIters, trainSmoothCoef);

        qDebug() << "loading FRGC";
        MultiBiomertricsAutoTuner::Input frgcData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithExportedCurvatureImages(frgcDirectory, "d", 300);

        qDebug() << "SoftKinetic data";
        MultiBiomertricsAutoTuner::Input softKineticData =
                MultiBiomertricsAutoTuner::Input::fromAlignedMeshes(ids, meshes);

        MultiBiomertricsAutoTuner::Settings settings(MultiBiomertricsAutoTuner::Settings::FCT_SVM, unitsFile);
        MultiExtractor extractor = MultiBiomertricsAutoTuner::train(frgcData, softKineticData, settings);

        extractor.serialize(outputDir);

        Evaluation eval = evaluateMultiExtractor(extractor, aligner, evaluationSoftKineticDir, testICPiters, testSmoothIters, testSmoothCoef);
        qDebug() << eval.eer;
        eval.outputResults(outputDir + "/eval", 10);
    }

    static void evaluateMultiExtractor()
    {
        MultiExtractor extractor("softKineticDoG");
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
        //QString dir("../../test/softKinetic/03/DS32528233700078_stepan/");
        QString dir("../../test/softKinetic/03/DS32528233700098_radim/");

        QVector<MultiTemplate> templates;
        QVector<QString> fileNames = Loader::listFiles(dir, "*.binz", Loader::Filename);
        foreach(const QString &fileName, fileNames) {
            Mesh m = Mesh::fromBINZ(dir + fileName);
            SurfaceProcessor::mdenoising(m, 0.02f, 10, 10);
            aligner.icpAlign(m, 100, FaceAligner::TemplateMatching);

            int id = fileName.split('-')[0].toInt();
            templates << extractor.extract(m, 1, id);
        }

        Evaluation evaluation = extractor.evaluate(templates);
        qDebug() << evaluation.eer;
    }
};

#endif // EVALUATESOFTKINETIC_H
