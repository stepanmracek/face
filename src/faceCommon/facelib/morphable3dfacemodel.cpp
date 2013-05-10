#include "morphable3dfacemodel.h"

#include "linalg/procrustes.h"
#include "facelib/surfaceprocessor.h"
#include "facelib/facealigner.h"
#include "landmarks.h"

Morphable3DFaceModel::Morphable3DFaceModel(const QString &pcaPathForZcoord, const QString &pcaPathForTexture, const QString &pcaFile, const QString &maskPath,
                                           const QString &landmarksPath, int width)
{
    pcaForZcoord = PCA(pcaPathForZcoord);
    pcaForTexture = PCA(pcaPathForTexture);
    pca = PCA(pcaFile);
    landmarks = Landmarks(landmarksPath);

    mask = Vector::fromFile(maskPath);
    int maskRows = mask.rows;
    Map faceDepth(width, maskRows/width);
    Map faceTexture(width, maskRows/width);
    for (int i = 0; i < maskRows; i++)
    {
        faceDepth.flags[i] = mask(i) ? true : false;
        faceTexture.flags[i] = mask(i) ? true : false;
    }

    Matrix zcoordMean = pcaForZcoord.getMean();
    Matrix textureMean = pcaForTexture.getMean();
    assert(zcoordMean.rows == faceDepth.flags.count(1));
    assert(zcoordMean.rows == textureMean.rows);
    int depthIndex = 0;
    for (int i = 0; i < maskRows; i++)
    {
        if (faceDepth.flags[i])
        {
            faceDepth.values[i] = zcoordMean(depthIndex);
            faceTexture.values[i] = textureMean(depthIndex);
            depthIndex++;
        }
    }

    mesh = Mesh::fromMap(faceDepth, faceTexture, true);
}

void Morphable3DFaceModel::setModelParams(Vector &commonParams)
{
    Vector backProjectedCommonParams = pca.backProject(commonParams);
    Vector zcoordParams(pcaForZcoord.getModes());
    Vector textureParams(pcaForTexture.getModes());

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

void Morphable3DFaceModel::setModelParams(Vector &zcoordParams, Vector &textureParams)
{
    Vector newZcoords = pcaForZcoord.backProject(zcoordParams);
    Vector newIntensities = pcaForTexture.backProject(textureParams);
    int n = newZcoords.rows;
    assert(n == mesh.points.count());
    assert(n == newIntensities.rows);

    for (int i = 0; i < n; i++)
    {
        mesh.points[i].z = newZcoords(i);
        uchar intensity = newIntensities(i);
        mesh.colors[i] = Color(intensity, intensity, intensity);
    }
}

Procrustes3DResult Morphable3DFaceModel::align(Mesh &inputMesh, Landmarks &inputLandmarks, int iterations)
{
    Procrustes3DResult result;

    // centralize
    cv::Point3d shift = Procrustes3D::centralizedTranslation(inputLandmarks.points);
    Procrustes3D::translate(inputLandmarks.points, shift);
    inputMesh.translate(shift);

    for (int iteration = 0; iteration < iterations; iteration++)
    {
        // rotate
        Matrix rotation = Procrustes3D::getOptimalRotation(inputLandmarks.points, this->landmarks.points);
        Procrustes3D::transform(inputLandmarks.points, rotation);
        inputMesh.transform(rotation);
        result.rotations << rotation;

        // scale
        cv::Point3d scaleParams = Procrustes3D::getOptimalScale(inputLandmarks.points, this->landmarks.points);
        Procrustes3D::scale(inputLandmarks.points, scaleParams);
        inputMesh.scale(scaleParams);
        result.scaleParams << scaleParams;
    }

    return result;
}

void Morphable3DFaceModel::morphModel(Mesh &alignedMesh)
{
    MapConverter converter;
    Map depthmap = SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100), 1, ZCoord);
    Map intensities = SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100), 1, Texture_I);
    for (int i = 0; i < mask.rows; i++)
    {
        if (mask(i) == 0)
        {
            depthmap.flags[i] = false;
            intensities.flags[i] = false;
        }
        else if (!depthmap.flags[i])
        {
            qDebug() << "Morphable3DFaceModel::morphModel, Missing value in input aligned mesh";
            //TODO!
            //depthmap.flags[i] = true;
            //depthmap.values[i] = ???
        }
    }

    QVector<double> usedZValues = depthmap.getUsedValues();
    Vector inputZValues(usedZValues);
    Vector zcoordParams = pcaForZcoord.project(inputZValues);
    Vector normalizedZcoordParams = pcaForZcoord.normalizeParams(zcoordParams);

    /*QVector<double> usedIValues = intensities.getUsedValues();
    Vector inputIValues(usedIValues);
    Vector textureParams = pcaForTexture.project(inputIValues);
    Vector normalizedTextureParams = pcaForTexture.normalizeParams(textureParams);*/
    Vector textureParams(pcaForTexture.getModes()); // no texture generation => texture params are just zeros

    setModelParams(normalizedZcoordParams, textureParams);

    // direct copy of input mesh texture to the model
    Map textureR = SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100), 1, Texture_R);
    Map textureG = SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100), 1, Texture_G);
    Map textureB = SurfaceProcessor::depthmap(alignedMesh, converter, cv::Point2d(-100,-100), cv::Point2d(100,100), 1, Texture_B);
    int n = textureR.values.count();
    assert(n == mask.rows);
    mesh.colors.clear();
    for (int i = 0; i < n; i++)
    {
        if (textureR.flags[i] && mask(i))
        {
            mesh.colors << Color(textureB.values[i], textureG.values[i], textureR.values[i]);
        }
    }
}

Mesh Morphable3DFaceModel::morph(Mesh &inputMesh, int iterations)
{
    // reset model
    Vector zeroParams(pca.getModes());
    setModelParams(zeroParams);

    // instantiate aligner and move the reference face, such the nosetip is at (0,0,0)
    FaceAligner aligner(this->mesh);
    aligner.referenceFace.translate(-this->landmarks.get(Landmarks::Nosetip));
    aligner.icpAlign(inputMesh, iterations);
    inputMesh.translate(this->landmarks.get(Landmarks::Nosetip));

    // morph
    morphModel(inputMesh);
    Mesh result(mesh);
    //Procrustes3D::applyInversedProcrustesResult(inputLandmarks.points, procrustesResult);
    //Procrustes3D::applyInversedProcrustesResult(inputMesh.points, procrustesResult);
    //Procrustes3D::applyInversedProcrustesResult(result.points, procrustesResult);
    result.recalculateMinMax();
    inputMesh.recalculateMinMax();

    return result;
}

Mesh Morphable3DFaceModel::morph(Mesh &inputMesh, Landmarks &inputLandmarks, int iterations)
{
    Procrustes3DResult procrustesResult = align(inputMesh, inputLandmarks, iterations, true);
    morphModel(inputMesh);
    Mesh result(mesh);
    Procrustes3D::applyInversedProcrustesResult(inputLandmarks.points, procrustesResult);
    Procrustes3D::applyInversedProcrustesResult(inputMesh.points, procrustesResult);
    Procrustes3D::applyInversedProcrustesResult(result.points, procrustesResult);
    result.recalculateMinMax();
    inputMesh.recalculateMinMax();

    return result;
}

void Morphable3DFaceModel::align(QVector<Mesh> &meshes,
                                 QVector<VectorOfPoints> &controlPoints,
                                 int iterations, bool scale)
{
    int meshCount = meshes.count();

    // centralize
    for (int i = 0; i < meshCount; i++)
    {
        VectorOfPoints &pointcloud = controlPoints[i];
        cv::Point3d shift = Procrustes3D::centralizedTranslation(pointcloud);

        Procrustes3D::translate(pointcloud, shift);
        meshes[i].translate(shift);
    }

    VectorOfPoints meanShape = Procrustes3D::getMeanShape(controlPoints);
    qDebug() << "Initial variation:" << Procrustes3D::getShapeVariation(controlPoints, meanShape);

    for (int iteration = 0; iteration < iterations; iteration++)
    {
        // rotate
        Procrustes3DResult rotResult = Procrustes3D::SVDAlign(controlPoints);
        for (int i = 0; i < meshCount; i++)
        {
            meshes[i].transform(rotResult.rotations[i]);
        }
        meanShape = Procrustes3D::getMeanShape(controlPoints);
        qDebug() << "Iteration:" << iteration << "after SVD:" << Procrustes3D::getShapeVariation(controlPoints, meanShape);

        // scale
        if (scale)
        {
            for (int meshIndex = 0; meshIndex < meshCount; meshIndex++)
            {
                cv::Point3d scaleParams = Procrustes3D::getOptimalScale(controlPoints[meshIndex], meanShape);
                Procrustes3D::scale(controlPoints[meshIndex], scaleParams);
                meshes[meshIndex].scale(scaleParams);
            }
        }

        meanShape = Procrustes3D::getMeanShape(controlPoints);
        qDebug() << "Iteration:" << iteration << "after scaling:" << Procrustes3D::getShapeVariation(controlPoints, meanShape);

        /*MapConverter c;
        Map map = SurfaceProcessor::depthmap(meshes[0], c, 1, Texture);
        cv::imshow("face", map.toMatrix());
        cv::waitKey(0);*/
    }
}

void Morphable3DFaceModel::create(QVector<Mesh> &meshes, QVector<VectorOfPoints> &controlPoints, int iterations,
                                  const QString &pcaForZcoordFile, const QString &pcaForTextureFile, const QString &pcaFile,
                                  const QString &flagsFile, const QString &meanControlPointsFile,
                                  Map &mapMask, bool scale)
{
    align(meshes, controlPoints, iterations, scale);
    VectorOfPoints meanControlPoints = Procrustes3D::getMeanShape(controlPoints);
    Landmarks l(meanControlPoints);
    l.serialize(meanControlPointsFile);

    QVector<Map> depthMaps;
    QVector<Map> textureMaps;

    Map resultZcoordMap(mapMask.w, mapMask.h);
    resultZcoordMap.setAll(0);
    resultZcoordMap.add(mapMask);

    Map resultTextureMap(mapMask.w, mapMask.h);
    resultTextureMap.setAll(0);
    resultTextureMap.add(mapMask);

    for (int index = 0; index < meshes.count(); index++)
    {
        Mesh &mesh = meshes[index];
        MapConverter converter;
        Map depth = SurfaceProcessor::depthmap(mesh, converter,
                                               cv::Point2d(-mapMask.w/2, -mapMask.h/2),
                                               cv::Point2d(mapMask.w/2, mapMask.h/2),
                                               1.0, ZCoord);
        resultZcoordMap.add(depth);
        depthMaps.append(depth);

        Map texture = SurfaceProcessor::depthmap(mesh, converter,
                                               cv::Point2d(-mapMask.w/2, -mapMask.h/2),
                                               cv::Point2d(mapMask.w/2, mapMask.h/2),
                                               1.0, Texture_I);
        resultTextureMap.add(texture);
        textureMaps.append(texture);

        /*cv::imshow("depth", depth.toMatrix());
        cv::imshow("texture", texture.toMatrix());
        cv::waitKey(100);*/
    }

    resultZcoordMap.linearTransform(1.0/meshes.count(), 1.0);
    //Matrix resultMatrix = resultMap.toMatrix() * 255;
    //cv::imwrite(meanImageFile.toStdString(), resultMatrix);

    QVector<Vector> zcoordVectors;
    QVector<Vector> textureVectors;
    for (int index = 0; index < meshes.count(); index++)
    {
        Map &depth = depthMaps[index];
        SurfaceProcessor::smooth(depth, 1, 2);
        depth.flags = resultZcoordMap.flags;
        QVector<double> zcoords = depth.getUsedValues();
        Vector zcoordsVec(zcoords);
        zcoordVectors << zcoordsVec;

        Map &texture = textureMaps[index];
        texture.flags = resultTextureMap.flags;
        QVector<double> intensities = texture.getUsedValues();
        Vector intensitiesVec(intensities);
        textureVectors << intensitiesVec;

        assert(zcoords.count() == intensities.count());
    }

    PCA pcaForZcoord(zcoordVectors);
    pcaForZcoord.modesSelectionThreshold(0.95);
    pcaForZcoord.serialize(pcaForZcoordFile);

    PCA pcaForTexture(textureVectors);
    pcaForTexture.modesSelectionThreshold(0.95);
    pcaForTexture.serialize(pcaForTextureFile);

    QVector<Vector> projectedZcoords = pcaForZcoord.batchProject(zcoordVectors);
    QVector<Vector> projectedTextures = pcaForTexture.batchProject(textureVectors);
    QVector<Vector> commonParams;
    for (int i = 0; i < meshes.count(); i++)
    {
        QVector<double> common;
        common << projectedZcoords[i].toQVector();
        common << projectedTextures[i].toQVector();

        Vector commonVec(common);
        commonParams << commonVec;
    }
    PCA pca(commonParams);
    pca.modesSelectionThreshold(0.95);
    pca.serialize(pcaFile);

    int n = resultZcoordMap.flags.count();
    QVector<double> flags(n, 0.0);
    for (int i = 0; i < n; i++)
    {
        if (resultZcoordMap.flags[i])
        {
            flags[i] = 1.0;
        }
    }
    Vector flagsVec(flags);
    flagsVec.toFile(flagsFile);
}
