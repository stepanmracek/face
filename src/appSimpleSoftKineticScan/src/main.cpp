#include <softkinetic/ds325sensorfactory.h>
#include <faceCommon/facedata/surfaceprocessor.h>

int main(int argc, char *argv[])
{
    std::string haarPath = (argc >= 2) ? argv[1] : "/mnt/data/face/test/haar-face.xml";
    std::string lmDetectorPath = (argc >= 3) ? argv[2] : "/mnt/data/face/test/shape_predictor_68_face_landmarks.dat";
    Face::Sensors::SoftKinetic::DS325SensorFactory factory;
    Face::Sensors::SoftKinetic::IDS325Sensor::Ptr sensor = factory.create(haarPath, lmDetectorPath);
    sensor->scan();

    auto mesh = sensor->sensorData().processedScan.mesh.zLevelSelect(0);
    mesh.rotate(0, -0.5, 0);

    Face::FaceData::MapConverter converter;
    auto img = Face::FaceData::SurfaceProcessor::depthmap(mesh, converter, 2.0, Face::FaceData::SurfaceProcessor::Texture_I);
    cv::imshow("result", img.toMatrix());
    cv::waitKey();
}
