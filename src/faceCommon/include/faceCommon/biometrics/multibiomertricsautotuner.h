#ifndef MULTIBIOMERTRICSAUTOTUNER_H
#define MULTIBIOMERTRICSAUTOTUNER_H

#include "faceCommon/biometrics/multiextractor.h"
#include "faceCommon/biometrics/multitemplate.h"
#include "faceCommon/faceCommon.h"

namespace Face {
namespace Biometrics {

class FACECOMMON_EXPORTS MultiBiomertricsAutoTuner
{
public:
    class FACECOMMON_EXPORTS Input
    {
    private:
        Input();

    public:
        /**
         * Translate image type name to the vector of images. Each image in vector belongs
         * to the corresponding subject id in 'ids' list
         */
        std::vector<MultiExtractor::ImageData> imageData;

        /**
         * IDs of subject in the input
         */
        std::vector<int> ids;

        static Input fromAlignedMeshes(const std::vector<int> &ids, const std::vector<FaceData::Mesh> meshes);
        static Input fromDirectoryWithAlignedMeshes(const std::string &path, const std::string &idAndScanSeparator,
                                                    int maxCount = -1, int startIndex = 0);

        static Input fromDirectoryWithTextureImages(const std::string &path, const std::string& idAndScanSeparator,
                                                    int maxCount = -1, int startIndex = 0);
    };

    struct FACECOMMON_EXPORTS Settings
    {
        std::string fusionType;
        std::vector<std::string> params;

        Settings();
        Settings(const std::string &fusionType, const std::string &unitsParametersFile);
    };

    static void fillEvaluations(const Input &sourceDatabaseTrainData, const Input &targetDatabaseTrainData,
                                const Settings &settings, std::vector<Evaluation> &evaluations);

    static MultiExtractor::Ptr trainWithWrapper(const Input &sourceDatabaseTrainData,
                                                const Input &targetDatabaseTrainData,
                                                const Settings &settings);

    static MultiExtractor::Ptr trainAllUnits(const Input &sourceDatabaseTrainData,
                                             const Input &targetDatabaseTrainData,
                                             const Settings &settings);

    static MultiExtractor::Unit::Ptr trainUnit(const std::string &lineParams, const Input &sourceDatabaseTrainData,
                                               const Input &targetDatabaseTrainData, std::vector<Evaluation> &evaluations,
                                               int index);



};

}
}

#endif // MULTIBIOMERTRICSAUTOTUNER_H
