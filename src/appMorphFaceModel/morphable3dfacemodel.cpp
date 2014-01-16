#include "morphable3dfacemodel.h"

#include "linalg/procrustes.h"
#include "facelib/surfaceprocessor.h"
#include "facelib/facealigner.h"
#include "facelib/landmarks.h"

Morphable3DFaceModel::Morphable3DFaceModel(const QString &pcaPathForZcoord, const QString &pcaPathForTexture, const QString &pcaFile, const QString &maskPath,
                                           const QString &landmarksPath, int width)
{
    pcaForZcoord = Face::LinAlg::PCA(pcaPathForZcoord);
    pcaForTexture = Face::LinAlg::PCA(pcaPathForTexture);
    pca = Face::LinAlg::PCA(pcaFile);
    landmarks = Face::FaceData::Landmarks(landmarksPath);

    mask = Face::LinAlg::Vector::fromFile(maskPath);
    int maskRows = mask.rows;
    Face::FaceData::Map faceDepth(width, maskRows/width);
    Face::FaceData::Map faceTexture(width, maskRows/width);
    for (int i = 0; i < maskRows; i++)
    {
        faceDepth.flags(i/faceDepth.w, i%faceDepth.w) = mask(i) ? true : false;
        faceTexture.flags(i/faceTexture.w, i%faceTexture.w) = mask(i) ? true : false;
    }

    Matrix zcoordMean = pcaForZcoord.getMean();
    Matrix textureMean = pcaForTexture.getMean();
    assert(zcoordMean.rows == textureMean.rows);
    int depthIndex = 0;
    for (int i = 0; i < maskRows; i++)
    {
        if (faceDepth.flags(i/faceDepth.w, i%faceDepth.w))
        {
            faceDepth.values(i/faceDepth.w, i%faceDepth.w) = zcoordMean(depthIndex);
            faceTexture.values(i/faceTexture.w, i%faceTexture.w) = textureMean(depthIndex);
            depthIndex++;
        }
    }

    mesh = Face::FaceData::Mesh::fromMap(faceDepth, faceTexture, false);
    mesh.translate(cv::Point3d(-width/2, -width/2, 0));
}

void Morphable3DFaceModel::setModelParams(Face::LinAlg::Vector &commonParams)
{
    Face::LinAlg::Vector backProjectedCommonParams = pca.backProject(commonParams);
    Face::LinAlg::Vector zcoordParams(pcaForZcoord.getModes());
    Face::LinAlg::Vector textureParams(pcaForTexture.getModes());

    for (int i = 0; i < pcaForZcoord.getModes(); i++)
    {
        zcoordParams(i) = backProjectedCommonParams(i);
    }
    for (int i = 0; i < pcaForTexture.getModes(); i++)
    {
        textureParams(i) = backProjectedCommonParams(i + pcaForZcoord.getModes());
    }
    setModelParams(zcoordParams, textureParams);
}

void Morphable3DFaceModel::setModelParams(Face::LinAlg::Vector &zcoordParams, Face::LinAlg::Vector &textureParams)
{
    Face::LinAlg::Vector newZcoords = pcaForZcoord.backProject(zcoordParams);
    Face::LinAlg::Vector newIntensities = pcaForTexture.backProject(textureParams);
    int n = newZcoords.rows;
    assert(n == mesh.pointsMat.rows);
    assert(n == newIntensities.rows);

    for (int i = 0; i < n; i++)
    {
        mesh.pointsMat(i, 2) = newZcoords(i);
        uchar intensity = newIntensities(i);
        mesh.colors[i] = Face::FaceData::Mesh::Color(intensity, intensity, intensity);
    }
}

Face::LinAlg::Procrustes3DResult Morphable3DFaceModel::align(Face::FaceData::Mesh &inputMesh,
                                                             Face::FaceData::Landmarks &inputLandmarks,
                                                             int iterations, bool scale)
{
    Face::LinAlg::Procrustes3DResult result;

    // centralize
    cv::Point3d shift = Face::LinAlg::Procrustes3D::centralizedTranslation(inputLandmarks.points);
    Face::LinAlg::Procrustes3D::translate(inputLandmarks.points, shift);
    inputMesh.translate(shift);

    for (int iteration = 0; iteration < iterations; iteration++)
    {
        // rotate
        Matrix rotation = Face::LinAlg::Procrustes3D::getOptimalRotation(inputLandmarks.points, this->landmarks.points);
        Face::LinAlg::Procrustes3D::transform(inputLandmarks.points, rotation);
        inputMesh.transform(rotation);
        result.rotations << rotation;

        // scale
        if (scale)
        {
            cv::Point3d scaleParams = Face::LinAlg::Procrustes3D::getOptimalScale(inputLandmarks.points, this->landmarks.points);
            Face::LinAlg::Procrustes3D::scale(inputLandmarks.points, scaleParams);
            inputMesh.scale(scaleParams);
            result.scaleParams << scaleParams;
        }
        else
        {
            result.scaleParams << cv::Point3d(1, 1, 1);
        }
    }

    return result;
}

void Morphable3DFaceModel::morphModel(Face::FaceData::Mesh &alignedMesh)
{
    // coord to index: y*w + x;
    /*int x = i % w;
    int y = i / w;*/

    Face::FaceData::MapConverter converter;
    Face::FaceData::Map depthmap =
            Face::FaceData::SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100),
                                                       1, Face::FaceData::SurfaceProcessor::ZCoord);
    Face::FaceData::Map intensities =
            Face::FaceData::SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100),
                                                       1, Face::FaceData::SurfaceProcessor::Texture_I);
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask(i) == 0)
        {
            depthmap.flags(i/depthmap.w, i%depthmap.w) = false;
            intensities.flags(i/intensities.w, i%intensities.w) = false;
        }
        else if (!depthmap.flags(i/depthmap.w, i%depthmap.w))
        {
            qDebug() << "Morphable3DFaceModel::morphModel, Missing value in input aligned mesh";
            //TODO!
            //depthmap.flags[i] = true;
            //depthmap.values[i] = ???
        }
    }

    QVector<double> usedZValues = depthmap.getUsedValues();
    Face::LinAlg::Vector inputZValues(usedZValues);
    Face::LinAlg::Vector zcoordParams = pcaForZcoord.project(inputZValues);
    Face::LinAlg::Vector normalizedZcoordParams = pcaForZcoord.normalizeParams(zcoordParams, 1);

    /*QVector<double> usedIValues = intensities.getUsedValues();
    Vector inputIValues(usedIValues);
    Vector textureParams = pcaForTexture.project(inputIValues);
    Vector normalizedTextureParams = pcaForTexture.normalizeParams(textureParams);*/
    Face::LinAlg::Vector textureParams(pcaForTexture.getModes()); // no texture generation => texture params are just zeros

    setModelParams(normalizedZcoordParams, textureParams);

    // direct copy of input mesh texture to the model
    Face::FaceData::Map textureR = Face::FaceData::SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100),
                                                              cv::Point2d(100,100), 1, Face::FaceData::SurfaceProcessor::Texture_R);
    Face::FaceData::Map textureG = Face::FaceData::SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100),
                                                              cv::Point2d(100,100), 1, Face::FaceData::SurfaceProcessor::Texture_G);
    Face::FaceData::Map textureB = Face::FaceData::SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100),
                                                              cv::Point2d(100,100), 1, Face::FaceData::SurfaceProcessor::Texture_B);
    int n = textureR.w * textureR.h;
    assert(n == mask.rows);
    mesh.colors.clear();
    for (int i = 0; i < n; i++)
    {
        if (textureR.flags[i] && mask(i))
        {
            mesh.colors << Face::FaceData::Mesh::Color(textureB.values(i/textureB.w, i%textureB.w),
                                 textureG.values(i/textureG.w, i%textureG.w),
                                 textureR.values(i/textureR.w, i%textureR.w));
        }
    }
}

Face::FaceData::Mesh Morphable3DFaceModel::morph(Face::FaceData::Mesh &inputMesh, int iterations)
{
    Face::FaceData::Mesh result(inputMesh);
    return result;

    /*// reset model
    Vector zeroParams(pca.getModes());
    setModelParams(zeroParams);

    // instantiate aligner and move the reference face, such the nosetip is at (0,0,0)
    qDebug() << landmarks.get(Landmarks::Nosetip).x << landmarks.get(Landmarks::Nosetip).y << landmarks.get(Landmarks::Nosetip).z;

    Mesh reference(this->mesh);
    reference.translate(-this->landmarks.get(Landmarks::Nosetip));
    FaceAligner aligner(reference);
    Procrustes3DResult procrustesResult = aligner.icpAlignDeprecated(inputMesh, iterations);
    inputMesh.translate(this->landmarks.get(Landmarks::Nosetip));

    // morph
    morphModel(inputMesh);
    Mesh result(mesh);
    //Procrustes3D::applyInversedProcrustesResult(inputLandmarks.points, procrustesResult);
    //Procrustes3D::applyInversedProcrustesResult(inputMesh.points, procrustesResult);
    //Procrustes3D::applyInversedProcrustesResult(result.points, procrustesResult);
    result.recalculateMinMax();
    inputMesh.recalculateMinMax();

    return result;*/
}

Face::FaceData::Mesh Morphable3DFaceModel::morph(Face::FaceData::Mesh &inputMesh, Face::FaceData::Landmarks &inputLandmarks, int iterations)
{
    Face::LinAlg::Procrustes3DResult procrustesResult = align(inputMesh, inputLandmarks, iterations, false);
    morphModel(inputMesh);
    Face::FaceData::Mesh result(mesh);
    Face::LinAlg::Procrustes3D::applyInversedProcrustesResult(inputLandmarks.points, procrustesResult);
    Face::LinAlg::Procrustes3D::applyInversedProcrustesResult(inputMesh.pointsMat, procrustesResult);
    Face::LinAlg::Procrustes3D::applyInversedProcrustesResult(result.pointsMat, procrustesResult);
    result.recalculateMinMax();
    inputMesh.recalculateMinMax();

    return result;
}

void Morphable3DFaceModel::align(QVector<Face::FaceData::Mesh> &meshes,
                                 QVector<Face::FaceData::VectorOfPoints> &controlPoints,
                                 int iterations, bool scale, bool centralize)
{
    int meshCount = meshes.count();

    // centralize
    if (centralize)
    {
        for (int i = 0; i < meshCount; i++)
        {
            Face::FaceData::VectorOfPoints &landmarks = controlPoints[i];
            cv::Point3d shift = Face::LinAlg::Procrustes3D::centralizedTranslation(landmarks);

            Face::LinAlg::Procrustes3D::translate(landmarks, shift);
            meshes[i].translate(shift);
        }
    }

    Face::FaceData::VectorOfPoints meanShape = Face::LinAlg::Procrustes3D::getMeanShape(controlPoints);
    qDebug() << "Initial variation:" << Face::LinAlg::Procrustes3D::getShapeVariation(controlPoints, meanShape);

    for (int iteration = 0; iteration < iterations; iteration++)
    {
        // rotate
        Face::LinAlg::Procrustes3DResult rotResult = Face::LinAlg::Procrustes3D::SVDAlign(controlPoints);
        for (int i = 0; i < meshCount; i++)
        {
            meshes[i].transform(rotResult.rotations[i]);
        }
        meanShape = Face::LinAlg::Procrustes3D::getMeanShape(controlPoints);
        qDebug() << "Iteration:" << iteration << "after SVD:" << Face::LinAlg::Procrustes3D::getShapeVariation(controlPoints, meanShape);

        // scale
        if (scale)
        {
            for (int meshIndex = 0; meshIndex < meshCount; meshIndex++)
            {
                cv::Point3d scaleParams = Face::LinAlg::Procrustes3D::getOptimalScale(controlPoints[meshIndex], meanShape);
                Face::LinAlg::Procrustes3D::scale(controlPoints[meshIndex], scaleParams);
                meshes[meshIndex].scale(scaleParams);
            }

            meanShape = Face::LinAlg::Procrustes3D::getMeanShape(controlPoints);
            qDebug() << "Iteration:" << iteration << "after scaling:" << Face::LinAlg::Procrustes3D::getShapeVariation(controlPoints, meanShape);
        }

        /*MapConverter c;
        Map map = SurfaceProcessor::depthmap(meshes[0], c, 1, Texture);
        cv::imshow("face", map.toMatrix());
        cv::waitKey(0);*/
    }
}

void Morphable3DFaceModel::create(QVector<Face::FaceData::Mesh> &meshes, QVector<Face::FaceData::VectorOfPoints> &controlPoints,
                                  int iterations, const QString &pcaForZcoordFile, const QString &pcaForTextureFile,
                                  const QString &pcaFile, const QString &flagsFile, const QString &meanControlPointsFile,
                                  Face::FaceData::Map &mapMask, bool scale, bool centralize)
{
    align(meshes, controlPoints, iterations, scale, centralize);
    Face::FaceData::VectorOfPoints meanControlPoints = Face::LinAlg::Procrustes3D::getMeanShape(controlPoints);
    Face::FaceData::Landmarks l(meanControlPoints);
    l.serialize(meanControlPointsFile);

    QVector<Face::FaceData::Map> depthMaps;
    QVector<Face::FaceData::Map> textureMaps;

    Face::FaceData::Map resultZcoordMap(mapMask.w, mapMask.h);
    resultZcoordMap.setAll(0);
    resultZcoordMap.add(mapMask);

    Face::FaceData::Map resultTextureMap(mapMask.w, mapMask.h);
    resultTextureMap.setAll(0);
    resultTextureMap.add(mapMask);

    qDebug() << "Creating depthmaps and textures";
    for (int index = 0; index < meshes.count(); index++)
    {
        Face::FaceData::Mesh &mesh = meshes[index];
        Face::FaceData::MapConverter converter;
        Face::FaceData::Map depth = Face::FaceData::SurfaceProcessor::depthmap(mesh, converter,
                                               cv::Point2d(-mapMask.w/2, -mapMask.h/2),
                                               cv::Point2d(mapMask.w/2, mapMask.h/2),
                                               1.0, Face::FaceData::SurfaceProcessor::ZCoord);
        resultZcoordMap.add(depth);
        depthMaps.append(depth);

        Face::FaceData::Map texture = Face::FaceData::SurfaceProcessor::depthmap(mesh, converter,
                                               cv::Point2d(-mapMask.w/2, -mapMask.h/2),
                                               cv::Point2d(mapMask.w/2, mapMask.h/2),
                                               1.0, Face::FaceData::SurfaceProcessor::Texture_I);
        resultTextureMap.add(texture);
        textureMaps.append(texture);

        /*cv::imshow("depth", depth.toMatrix());
        cv::imshow("texture", texture.toMatrix());
        cv::waitKey(100);*/
    }

    resultZcoordMap.linearTransform(1.0/meshes.count(), 1.0);
    //Matrix resultMatrix = resultMap.toMatrix() * 255;
    //cv::imwrite(meanImageFile.toStdString(), resultMatrix);

    qDebug() << "Creating input projection vectors for depthmaps and textures";
    QVector<Face::LinAlg::Vector> zcoordVectors;
    QVector<Face::LinAlg::Vector> textureVectors;
    for (int index = 0; index < meshes.count(); index++)
    {
        Face::FaceData::Map &depth = depthMaps[index];
        //SurfaceProcessor::smooth(depth, 1, 2);
        depth.flags = resultZcoordMap.flags;
        QVector<double> zcoords = depth.getUsedValues();
        Face::LinAlg::Vector zcoordsVec(zcoords);
        zcoordVectors << zcoordsVec;

        Face::FaceData::Map &texture = textureMaps[index];
        texture.flags = resultTextureMap.flags;
        QVector<double> intensities = texture.getUsedValues();
        Face::LinAlg::Vector intensitiesVec(intensities);
        textureVectors << intensitiesVec;

        assert(zcoords.count() == intensities.count());
    }

    qDebug() << "PCA learning";
    Face::LinAlg::PCA pcaForZcoord(zcoordVectors);
    pcaForZcoord.modesSelectionThreshold(0.95);
    pcaForZcoord.serialize(pcaForZcoordFile);

    Face::LinAlg::PCA pcaForTexture(textureVectors);
    pcaForTexture.modesSelectionThreshold(0.95);
    pcaForTexture.serialize(pcaForTextureFile);

    QVector<Face::LinAlg::Vector> projectedZcoords = pcaForZcoord.batchProject(zcoordVectors);
    QVector<Face::LinAlg::Vector> projectedTextures = pcaForTexture.batchProject(textureVectors);
    QVector<Face::LinAlg::Vector> commonParams;
    for (int i = 0; i < meshes.count(); i++)
    {
        QVector<double> common;
        common << projectedZcoords[i].toQVector();
        common << projectedTextures[i].toQVector();

        Face::LinAlg::Vector commonVec(common);
        commonParams << commonVec;
    }
    Face::LinAlg::PCA pca(commonParams);
    pca.modesSelectionThreshold(0.95);
    pca.serialize(pcaFile);

    int n = resultZcoordMap.w * resultZcoordMap.h;
    QVector<double> flags(n, 0.0);
    for (int i = 0; i < n; i++)
    {
        if (resultZcoordMap.flags[i])
        {
            flags[i] = 1.0;
        }
    }
    Face::LinAlg::Vector flagsVec(flags);
    flagsVec.toFile(flagsFile);
}
