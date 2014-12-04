#include <softkinetic/ds325sensorfactory.h>
#include <faceCommon/facedata/surfaceprocessor.h>

int main(int argc, char *argv[])
{
    std::string haarPath = (argc >= 2) ? argv[1] : "../../test/haar-face.xml";
    Face::Sensors::SoftKinetic::DS325SensorFactory factory;
    auto sensor = factory.create(haarPath);
    sensor->scan();

    auto mesh = sensor->mesh().zLevelSelect(0);
    mesh.rotate(0, -0.5, 0);

    Face::FaceData::MapConverter converter;
    auto img = Face::FaceData::SurfaceProcessor::depthmap(mesh, converter, 2.0, Face::FaceData::SurfaceProcessor::Texture_I);
    cv::imshow("result", img.toMatrix());
    cv::waitKey();
}
