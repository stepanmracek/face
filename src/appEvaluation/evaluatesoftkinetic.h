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

    static void createMultiExtractor()
    {
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
        QString dir("../../test/softKinetic/03/DS32528233700078_stepan/");

        QVector<int> ids;
        QVector<Mesh> meshes;
        QVector<QString> fileNames = Loader::listFiles(dir, "*.binz", Loader::Filename);
        foreach(const QString &fileName, fileNames) {
            Mesh m = Mesh::fromBINZ(dir + fileName);
            SurfaceProcessor::mdenoising(m, 0.02f, 10, 10);
            aligner.icpAlign(m, 20, FaceAligner::TemplateMatching);

            ids << fileName.split('-')[0].toInt();
            meshes << m;
        }

        MultiBiomertricsAutoTuner::Input frgcData =
                MultiBiomertricsAutoTuner::Input::fromDirectoryWithExportedCurvatureImages("/home/stepo/data/frgc/spring2004/zbin-aligned2/", "d", 300);

        MultiBiomertricsAutoTuner::Input softKineticData =
                MultiBiomertricsAutoTuner::Input::fromAlignedMeshes(ids, meshes);

        MultiBiomertricsAutoTuner::Settings settings(MultiBiomertricsAutoTuner::FCT_SVM, "allUnits");

        MultiExtractor extractor = MultiBiomertricsAutoTuner::train(frgcData, softKineticData, settings);
        extractor.serialize("softKinetic");
    }

    static void evaluateMultiExtractor()
    {
        MultiExtractor extractor("softKinetic");
        FaceAligner aligner(Mesh::fromOBJ("../../test/meanForAlign.obj", false));
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
