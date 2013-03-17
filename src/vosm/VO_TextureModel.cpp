/****************************************************************************
*                                                                           *
*   IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.       *
*                                                                           *
*   By downloading, copying, installing or using the software you agree to  *
*   this license. If you do not agree to this license, do not download,     *
*   install, copy or use the software.                                      *
*                                                                           *
*                           License Agreement                               *
*                   For Vision Open Statistical Models                      *
*                                                                           *
*   Copyright (C):      2006~2012 by JIA Pei, all rights reserved.          *
*                                                                           *
*   VOSM is free software under the terms of the GNU Lesser General Public  *
*   License (GNU LGPL) as published by the Free Software Foundation; either *
*   version 3.0 of the License, or (at your option) any later version.      *
*   You can use it, modify it, redistribute it, etc; and redistribution and *
*   use in source and binary forms, with or without modification, are       *
*   permitted provided that the following conditions are met:               *
*                                                                           *
*   a) Redistribution's of source code must retain this whole paragraph of  *
*   copyright notice, including this list of conditions and all the         *
*   following contents in this  copyright paragraph.                        *
*                                                                           *
*   b) Redistribution's in binary form must reproduce this whole paragraph  *
*   of copyright notice, including this list of conditions and all the      *
*   following contents in this copyright paragraph, and/or other materials  *
*   provided with the distribution.                                         *
*                                                                           *
*   c) The name of the copyright holders may not be used to endorse or      *
*   promote products derived from this software without specific prior      *
*   written permission.                                                     *
*                                                                           *
*   Any publications based on this code must cite the following five papers,*
*   technical reports and on-line materials.                                *
*   1) P. JIA, 2D Statistical Models, Technical Report of Vision Open       *
*   Working Group, 2st Edition, October 21, 2010.                           *
*   http://www.visionopen.com/members/jiapei/publications/pei_sm2dreport2010.pdf*
*   2) P. JIA. Audio-visual based HMI for an Intelligent Wheelchair.        *
*   PhD thesis, University of Essex, February, 2011.                        *
*   http://www.visionopen.com/members/jiapei/publications/pei_phdthesis2010.pdf*
*   3) T. Cootes and C. Taylor. Statistical models of appearance for        *
*   computer vision. Technical report, Imaging Science and Biomedical       *
*   Engineering, University of Manchester, March 8, 2004.                   *
*   http://www.isbe.man.ac.uk/~bim/Models/app_models.pdf                    *
*   4) I. Matthews and S. Baker. Active appearance models revisited.        *
*   International Journal of Computer Vision, 60(2):135--164, November 2004.*
*   http://www.ri.cmu.edu/pub_files/pub4/matthews_iain_2004_2/matthews_iain_2004_2.pdf*
*   5) M. B. Stegmann, Active Appearance Models: Theory, Extensions and     *
*   Cases, 2000.                                                            *
*   http://www2.imm.dtu.dk/~aam/main/                                       *
*                                                                           *
* Version:          0.4                                                     *
* Author:           JIA Pei                                                 *
* Contact:          jp4work@gmail.com                                       *
* URL:              http://www.visionopen.com                               *
* Create Date:      2008-04-03                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/


#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <boost/filesystem.hpp>

// Explained by JIA Pei. 2009-07-31. #include <windows.h> is a must for <GL/gl.h> and <GL.glu.h>
//#ifdef _WIN32
//#include <windows.h>
//#endif
//#include <GL/gl.h>
//#include <GL/glu.h>
//#include <GL/freeglut.h>

#include "VO_TextureModel.h"
#include "VO_CVCommon.h"


/** Default Constructor */
VO_TextureModel::VO_TextureModel()
{
    this->init();
}


/** Initialization */
void VO_TextureModel::init()
{
    this->m_iTextureRepresentationMethod        = VO_Features::DIRECT;
    this->m_iNbOfTextureRepresentations         = 0;
    this->m_iNbOfChannels                       = 1;
    this->m_iNbOfPixels                         = 0;
    this->m_iNbOfTextures                       = 0;
    this->m_iNbOfEigenTexturesAtMost            = 0;
    this->m_iNbOfTextureEigens                  = 0;
    this->m_fAverageTextureStandardDeviation    = 0.0f;
    this->m_fTruncatedPercent_Texture           = 0.95f;
    this->m_vTextures.clear();
    this->m_vNormalizedTextures.clear();
    this->m_vTemplatePointWarpInfo.clear();
    this->m_vNormalizedPointWarpInfo.clear();
}


VO_TextureModel::~VO_TextureModel()
{
    this->m_vTextures.clear();
    this->m_vNormalizedTextures.clear();
    this->m_vTemplatePointWarpInfo.clear();
    this->m_vNormalizedPointWarpInfo.clear();
}


/**
 * @author         JIA Pei
 * @version        2010-02-10
 * @brief          Calculate point warping information
 * @param          triangles           Input - a vector of all triangles
 * @param          ch                  Input - convex hull
 * @param          warpInfo            Input - information of a vector of warping points
 * @return         unsigned int        points (actually pixels) inside the template face
 * @note           Obviously, triangles and ch contain duplicated information
*/
unsigned int VO_TextureModel::VO_CalcPointWarpingInfo(const vector <VO_Triangle2DStructure>& templateTriangles, vector<VO_WarpingPoint>& warpInfo)
{
    unsigned int NbOfPixels = 0;

    Rect rect               = VO_TextureModel::VO_CalcBoundingRectFromTriangles(templateTriangles);

    for (unsigned int i = 0; i < rect.height; i++)
    {
        for (unsigned int j = 0; j < rect.width; j++)
        {
            // JIA Pei. 2006-11-25. You will see the following (int) is very important
            // without (int), the result here is not correct at all!!
            Point2f pt = Point2f((float)j, (float)i);

            int k = VO_Triangle2DStructure::IsPointInTriangles(pt,templateTriangles);
            if(k>=0)
            {
                VO_WarpingPoint tempPixelTriangle;

                // necessary for all methods
                tempPixelTriangle.SetPosition(pt);
                tempPixelTriangle.SetTriangleIndex(k);
                tempPixelTriangle.SetPointIndex(NbOfPixels);
                tempPixelTriangle.SetTriangle2DStructure(templateTriangles[k] );

                // Very important!! Note by JIA Pei, push_back cannot perform on 2-D vectors
                warpInfo.push_back (tempPixelTriangle);

                NbOfPixels ++;
            }
        }
    }

    return NbOfPixels;
}


/**
* @author       JIA Pei, YAO Wei
 * @version     2010-02-10
 * @brief       Load one VO_Texture from an image based on VO_Shape
 * @param       iShape                  Input     - the shape
 * @param       img                     Input     - image
 * @param       templateTriangles       Input     - the composed face template triangles
 * @param       warpInfo                Input     - warping information for all pixels in template face
 * @param       oTexture                Output    - the extracted texture
 * @param       trm                     Input     - texture representation method
 * @return      bool                    loading succeed or not?
*/
bool VO_TextureModel::VO_LoadOneTextureFromShape(const VO_Shape& iShape, 
                                                const Mat& img,
                                                const vector<VO_Triangle2DStructure>& templateTriangles,
                                                const vector<VO_WarpingPoint>& warpInfo,
                                                VO_Texture& oTexture, 
                                                int trm)
{
    ofstream fs;
    fs.open("loadTextureTime.txt", ios::app);

double time0 = (double)cvGetTickCount();

    // make sure all shape points are inside the image
    if ( !VO_ShapeModel::VO_IsShapeInsideImage(iShape, img) )
    {
        cout << "The shape goes out of the image" << endl;
        cout << "Shape =" << iShape << endl;
        cout << "cols=" << img.cols << " rows=" << img.rows << endl;

        return false;
    }


    unsigned int NbOfShapeDim   = iShape.GetNbOfDim();
    unsigned int NbOfPixels     = warpInfo.size();
    unsigned int NbOfChannels   = img.channels();
    unsigned int NbOfTextures   = NbOfPixels*NbOfChannels;
    unsigned int NbOfTriangles  = templateTriangles.size();
    oTexture.m_MatTexture       = Mat_<float>::zeros(NbOfChannels, NbOfPixels);

    Point2f src[3], dst[3];
    vector< Mat_<float> > matWarping;
    matWarping.resize(NbOfTriangles);


    // calculate all the mapping (for speeding up) 95 mapping totally
    for (unsigned int j = 0; j < NbOfTriangles; j++ )
    {
        // Get the affine transformation for each triangle pair.
        src[0] = templateTriangles[j].GetA2DPoint(0);
        src[1] = templateTriangles[j].GetA2DPoint(1);
        src[2] = templateTriangles[j].GetA2DPoint(2);

        dst[0] = iShape.GetA2DPoint(templateTriangles[j].m_vVertexIndexes[0]);
        dst[1] = iShape.GetA2DPoint(templateTriangles[j].m_vVertexIndexes[1]);
        dst[2] = iShape.GetA2DPoint(templateTriangles[j].m_vVertexIndexes[2]);

        matWarping[j] = cv::getAffineTransform( src, dst );
    }

    Rect rect = iShape.GetShapeBoundRect();
    // Why +1? A must.
    // The bounded rectangle could be (0, 0, 2, 2)
    // That means the width of the image is 2-0+1=3 (from the aspect of pixel)
    rect.width +=1; rect.height +=1;

double time1 = (double)cvGetTickCount();
double elapsed = (time1 -  time0 )/  (cvGetTickFrequency()*1000.);
fs << "Before Mapping -- Step 1 of warping time: " << elapsed << "millisec."  << endl;

    Mat Img2BExtracted;

    switch(trm)
    {
    case VO_Features::LAPLACE:
        {
            Mat tempImg2BExtracted = Mat::zeros(img.size(), CV_32F);
            img.convertTo( tempImg2BExtracted, tempImg2BExtracted.type() );

            // Explained by JIA Pei, 2008-03-09.
            // Why I don't do cvCopy(tempImg2BExtracted, tempExcavated); right here,
            // but after cvSmooth, cvLaplace, cvAbs?
            // Because we may deal with the boundary and corners much better in this way.
            cv::GaussianBlur( tempImg2BExtracted, tempImg2BExtracted, Size(5, 5), 0.5, 0.5);
            cv::Laplacian( tempImg2BExtracted, tempImg2BExtracted, tempImg2BExtracted.depth(), 3);
            cv::abs(tempImg2BExtracted);

            Img2BExtracted = tempImg2BExtracted(rect);
        }
        break;
    case VO_Features::HARRISCORNER:
        {
            Mat tempImg2BExtracted;

            // Explained by JIA Pei, 2008-03-09.
            // Why I don't do cvCopy(tempImg2BExtracted, tempExcavated); right here,
            // but after cvCornerHarris?
            // Because we may deal with the boundary and corners much better in this way.
            // cvCornerHarris is for one channel

            switch (NbOfChannels)
            {
            case GRAYCHANNELS:
                {
                    cv::cornerHarris( img, tempImg2BExtracted, 3, 3, 0.02);
                    cv::abs(tempImg2BExtracted);
                }
                break;
            case COLORCHANNELS:
            default:
                {
                    vector<Mat> bgr, bgrHC;
                    bgrHC.resize(3);
                    cv::split(img, bgr);

                    for(unsigned int i = 0; i < 3; i++)
                    {
                        cv::cornerHarris( bgr[i], bgrHC[i], 3, 3, 0.02);
                    }

                    cv::merge(bgrHC, tempImg2BExtracted);
                    cv::abs(tempImg2BExtracted);
                }
                break;
            }
            
            Img2BExtracted = tempImg2BExtracted(rect);
        }
        break;
    case VO_Features::GABOR: // This might be very slow
        {
        }
        break;
    case VO_Features::SEGMENTATION:
        {
            Mat tempImg2BExtracted = Mat::zeros(img.size(), CV_32F);
            img.convertTo( tempImg2BExtracted, tempImg2BExtracted.type() );
            
            cv::GaussianBlur( tempImg2BExtracted, tempImg2BExtracted, Size(5, 5), 0.5, 0.5);
            Img2BExtracted = tempImg2BExtracted(rect );

            cv::threshold( Img2BExtracted, Img2BExtracted, 0, 255, THRESH_BINARY + THRESH_OTSU );    // only for single channel images
        }
        break;
    case VO_Features::HISTOGRAMEQUALIZED:
        {
            Img2BExtracted = img(rect);

            switch (NbOfChannels)
            {
            case GRAYCHANNELS:
                cv::equalizeHist( Img2BExtracted, Img2BExtracted );
                break;
            case COLORCHANNELS:
            default:
                {
                    vector<Mat> bgr;
                    cv::split(Img2BExtracted, bgr);

                    for(unsigned int i = 0; i < 3; i++)
                        cv::equalizeHist( bgr[i], bgr[i] );

                    cv::merge(bgr, Img2BExtracted);
                }
                break;
            }
        }
        break;
    case VO_Features::DIRECT:
        {
            Img2BExtracted = img(rect);
        }
    default:
        break;
    }

double time2 = (double)cvGetTickCount();
elapsed = (time2 - time1)/  (cvGetTickFrequency()*1000.);
fs << "Mapping -- Step 2 of warping time: " << elapsed << "millisec."  << endl;

    //Mat Img4Display = Mat::zeros(Img2BExtracted.size(), CV_8U);
    //Img2BExtracted.convertTo(Img4Display, Img4Display.type());
    //imwrite("testtest.jpg", Img4Display);

    float x;
    float y;
    register int X;
    register int Y ;
    register int X1;
    register int Y1;
    register float s;
    register float t;
    register float s1;
    register float t1;
    float XX, YY;

    if(Img2BExtracted.type() == CV_32FC3)
    {
        for (unsigned int i = 0; i < NbOfPixels; i++ )
        {
            // Explained by Yao Wei. This part causes the slow speed of the warping.
//            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
//            warpsrc(0, 0) = warpInfo[i].m_CVPosition.x;
//            warpsrc(1, 0) = warpInfo[i].m_CVPosition.y;
//            warpsrc(2, 0) = 1.0;
////          cv::gemm(matWarping[warpInfo[i].m_iTriangleIndex], warpsrc, 1.0, Mat(), 0.0, warpeddst);
//            warpeddst = matWarping[warpInfo[i].m_iTriangleIndex] * warpsrc;
            
            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
            const Mat_<float>& sss = matWarping[warpInfo[i].m_iTriangleIndex];
            XX = sss(0,0)*warpInfo[i].m_CVPosition.x + sss(0,1)*warpInfo[i].m_CVPosition.y
                +sss(0,2);
            YY = sss(1,0)*warpInfo[i].m_CVPosition.x + sss(1,1)*warpInfo[i].m_CVPosition.y
                +sss(1,2);

            // Revised by Yao Wei. Now, much much faster !!!
            x = XX - rect.x;
            y = YY - rect.y;
            X = cvFloor(x);
            Y = cvFloor(y);
            X1 = cvCeil(x);
            Y1 = cvCeil(y);
            s=x-X;
            t=y-Y;
            s1 = 1.0f - s;
            t1 = 1.0f - t;
            
            for (int j = 0; j < 3; j++)
            {
                oTexture.m_MatTexture(j, i) =
                ( t1 * Img2BExtracted.at<Vec3f>(Y, X)[j] +
                t * Img2BExtracted.at<Vec3f>(Y1, X)[j] ) * s1 +
                ( t1 * Img2BExtracted.at<Vec3f>(Y, X1)[j] +
                t * Img2BExtracted.at<Vec3f>(Y1, X1)[j] ) * s;
            }
        }
    }
    else if(Img2BExtracted.type() == CV_8UC3)
    {
        for (unsigned int i = 0; i < NbOfPixels; i++ )
        {
            // Explained by Yao Wei. This part causes the slow speed of the warping.
//            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
//            warpsrc(0, 0) = warpInfo[i].m_CVPosition.x;
//            warpsrc(1, 0) = warpInfo[i].m_CVPosition.y;
//            warpsrc(2, 0) = 1.0;
////            cv::gemm(matWarping[warpInfo[i].m_iTriangleIndex], warpsrc, 1.0, Mat(), 0.0, warpeddst);
//            warpeddst = matWarping[warpInfo[i].m_iTriangleIndex] * warpsrc;

            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
            const Mat_<float>& sss = matWarping[warpInfo[i].m_iTriangleIndex];
            XX = sss(0,0)*warpInfo[i].m_CVPosition.x + sss(0,1)*warpInfo[i].m_CVPosition.y
                +sss(0,2);
            YY = sss(1,0)*warpInfo[i].m_CVPosition.x + sss(1,1)*warpInfo[i].m_CVPosition.y
                +sss(1,2);

            // Revised by Yao Wei. Now, much much faster !!!
            x = XX - rect.x;
            y = YY - rect.y;
            X = cvFloor(x);
            Y = cvFloor(y);
            X1 = cvCeil(x);
            Y1 = cvCeil(y);
            s=x-X;
            t=y-Y;
            s1 = 1.0f - s;
            t1 = 1.0f - t;

            for (int j = 0; j < 3; j++)
            {
                oTexture.m_MatTexture(j, i) =
                ( t1 * Img2BExtracted.at<Vec3b>(Y, X)[j] +
                t * Img2BExtracted.at<Vec3b>(Y1, X)[j] ) * s1 +
                ( t1 * Img2BExtracted.at<Vec3b>(Y, X1)[j] +
                t * Img2BExtracted.at<Vec3b>(Y1, X1)[j] ) * s;
            }
        }
    }
    else if(Img2BExtracted.type() == CV_32FC1)
    {
        for (unsigned int i = 0; i < NbOfPixels; i++ )
        {
            // Explained by Yao Wei. This part causes the slow speed of the warping.
//            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
//            warpsrc(0, 0) = warpInfo[i].m_CVPosition.x;
//            warpsrc(1, 0) = warpInfo[i].m_CVPosition.y;
//            warpsrc(2, 0) = 1.0;
////          cv::gemm(matWarping[warpInfo[i].m_iTriangleIndex], warpsrc, 1.0, Mat(), 0.0, warpeddst);
//            warpeddst = matWarping[warpInfo[i].m_iTriangleIndex] * warpsrc;

            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
            const Mat_<float>& sss = matWarping[warpInfo[i].m_iTriangleIndex];
            XX = sss(0,0)*warpInfo[i].m_CVPosition.x + sss(0,1)*warpInfo[i].m_CVPosition.y
                +sss(0,2);
            YY = sss(1,0)*warpInfo[i].m_CVPosition.x + sss(1,1)*warpInfo[i].m_CVPosition.y
                +sss(1,2);

            // Revised by Yao Wei. Now, much much faster !!!
            x = XX - rect.x;
            y = YY - rect.y;
            X = cvFloor(x);
            Y = cvFloor(y);
            X1 = cvCeil(x);
            Y1 = cvCeil(y);
            s=x-X;
            t=y-Y;
            s1 = 1.0f - s;
            t1 = 1.0f - t;

            oTexture.m_MatTexture(0, i) =
            ( t1 * Img2BExtracted.at<float>(Y, X) +
            t * Img2BExtracted.at<float>(Y1, X) ) * s1 +
            ( t1 * Img2BExtracted.at<float>(Y, X1) +
            t * Img2BExtracted.at<float>(Y1, X1) ) * s;
        }
    }
    else if(Img2BExtracted.type() == CV_8UC1)
    {
        for (unsigned int i = 0; i < NbOfPixels; i++ )
        {
            // Explained by Yao Wei. This part causes the slow speed of the warping.
//            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
//            warpsrc(0, 0) = warpInfo[i].m_CVPosition.x;
//            warpsrc(1, 0) = warpInfo[i].m_CVPosition.y;
//            warpsrc(2, 0) = 1.0;
////            cv::gemm(matWarping[warpInfo[i].m_iTriangleIndex], warpsrc, 1.0, Mat(), 0.0, warpeddst);
//            warpeddst = matWarping[warpInfo[i].m_iTriangleIndex] * warpsrc;
            

            // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
            const Mat_<float>& sss = matWarping[warpInfo[i].m_iTriangleIndex];
            XX = sss(0,0)*warpInfo[i].m_CVPosition.x + sss(0,1)*warpInfo[i].m_CVPosition.y
                +sss(0,2);
            YY = sss(1,0)*warpInfo[i].m_CVPosition.x + sss(1,1)*warpInfo[i].m_CVPosition.y
                +sss(1,2);

            // Revised by Yao Wei. Now, much much faster !!!
            x = XX - rect.x;
            y = YY - rect.y;
            X = cvFloor(x);
            Y = cvFloor(y);
            X1 = cvCeil(x);
            Y1 = cvCeil(y);
            s=x-X;
            t=y-Y;
            s1 = 1.0f - s;
            t1 = 1.0f - t;

            oTexture.m_MatTexture(0, i) =
            ( t1 * Img2BExtracted.at<uchar>(Y, X) +
            t * Img2BExtracted.at<uchar>(Y1, X) ) * s1 +
            ( t1 * Img2BExtracted.at<uchar>(Y, X1) +
            t * Img2BExtracted.at<uchar>(Y1, X1) ) * s;
        }
    }
double time3 = (double)cvGetTickCount();
elapsed = (time3 - time2)/  (cvGetTickFrequency()*1000.);
fs << "Mapping -- Step 3 of warping time: " << elapsed << "millisec."  << endl;

//Rect rect1                   = VO_TextureModel::VO_CalcBoundingRectFromTriangles(templateTriangles);
//Mat otmpImg;
//if(NbOfChannels == 1)
//    otmpImg = Mat::zeros(Size(rect1.width, rect1.height), CV_8UC1);
//else if(NbOfChannels == 3)
//    otmpImg = Mat::zeros(Size(rect1.width, rect1.height), CV_8UC3);
//VO_TextureModel::VO_PutOneTextureToTemplateShape(oTexture, templateTriangles, otmpImg);
//imwrite("ttttt.jpg", otmpImg);
///** How many triangles totally */
//string trianglestr;
//stringstream ssi;
//string stri;
//static int NbOfImages = 0;
//ssi << NbOfImages;
//ssi >> stri;
//if(stri.length() == 2)
//    trianglestr = "0" + stri + ".jpg";
//else if(stri.length() == 1)
//    trianglestr = "00" + stri + ".jpg";
//else
//    trianglestr = stri + ".jpg";
//
//imwrite(trianglestr.c_str(), otmpImg);
//
//ssi.clear();
//NbOfImages++;

    fs.close();
    return true;

}


/**
* @author     JIA Pei
* @version    2010-02-10
* @brief      Normalize all textures - 2-norm all to "1"
* @param      vTextures            Input   - all textures before normalization
* @param      normalizedTextures   Output  - all textures after normalization
* @return     return               float   average texture size of all textures
*/
float VO_TextureModel::VO_NormalizeAllTextures(const vector<VO_Texture>& vTextures, vector<VO_Texture>& normalizedTextures)
{
    unsigned int NbOfSamples                    = vTextures.size();
    unsigned int NbOfPixels                       = vTextures[0].GetNbOfPixels();
    float averageTextureStandardDeviation       = 0.0f;

    normalizedTextures = vTextures;

    for(unsigned int i = 0; i < NbOfSamples; i++)
    {
        averageTextureStandardDeviation += normalizedTextures.at(i).GetStandardizedTextureNorm();
        normalizedTextures[i].Normalize();
    }
    averageTextureStandardDeviation /= (float)NbOfSamples;

    return averageTextureStandardDeviation;
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Rescale the normalizetextures so that meantexture's 2-norm could be "1"
 * @param       meanNormalizeTexture    Input - meannormalizedtexture that all normalizedtextures should rescale to
 * @param       normalizedTextures      Input and Output - normalized texture and normalized rescaled texture
 * @return      void
*/
void VO_TextureModel::VO_RescaleAllNormalizedTextures(  const VO_Texture& meanNormalizeTexture,
                                                        vector<VO_Texture>& normalizedTextures)
{
    unsigned int NbOfSamples = normalizedTextures.size();

    // Explained by JIA Pei. 2010-02-10. scale back so that the mean shape size is equal to 1.0
    for(unsigned int i = 0; i < NbOfSamples; i++)
    {
        VO_TextureModel::VO_RescaleOneNormalizedTexture( meanNormalizeTexture, normalizedTextures[i] );
    }
}


/**
 * @author         JIA Pei
 * @version        2010-02-20
 * @brief          Rescale the normalizedTexture to the already trained meanNormalizeTexture
 * @param          meanNormalizeTexture    Input - meanNormalizeTexture that all normalizedTextures should rescale to
 * @param          normalizedTexture       Input and Output - normalized texture and normalized rescaled texture
*/
void VO_TextureModel:: VO_RescaleOneNormalizedTexture(const VO_Texture& meanNormalizeTexture,
                                                        VO_Texture& normalizedTexture)
{
    float ts = sqrt ( fabs(normalizedTexture.dot(meanNormalizeTexture) ) );
    normalizedTexture.Scale( 1.0f/ts );
}


/**
 * @author         JIA Pei
 * @version        2010-02-10
 * @brief          Calculate mean texture
 * @param          vTextures         Input     - all textures
 * @param          meanTexture       Output     - mean texture
*/
void VO_TextureModel::VO_CalcMeanTexture(const vector<VO_Texture>& vTextures, VO_Texture& meanTexture)
{
    unsigned int NbOfSamples    = vTextures.size();
    meanTexture = vTextures[0];

    for(unsigned int i = 1; i < NbOfSamples; i++)
    {
        meanTexture += vTextures[i];
    }

    meanTexture /= (float)NbOfSamples;
}


/**
 * @author         JIA Pei
 * @version        2010-02-10
 * @brief          Put one texture into the template shape with a corresponding warp
 * @param          texture         Input
 * @param          triangles       Input     - template triangle, contains warping information
 * @param          oImg            Output
 * @note           All texture vectors are of the same size. So, you can't put it into the respective triangulation
                for respective image. You've got to put a texture into a template shape first, then VO_WarpFromOneShapeToAnother
 * @return        void
*/
void VO_TextureModel::VO_PutOneTextureToTemplateShape(const VO_Texture& texture, const vector <VO_Triangle2DStructure>& triangles, Mat& oImg)
{
    unsigned int NbOfChannels = texture.GetNbOfTextureRepresentation() >= 3 ? 3:1;
    VO_Shape shape = VO_Triangle2DStructure::Triangle2D2Shape(triangles);
    Rect rect = shape.GetShapeBoundRect();
    if(NbOfChannels == 1)
        oImg = Mat::zeros(Size(rect.width, rect.height), CV_8UC1);
    else if(NbOfChannels == 3)
        oImg = Mat::zeros(Size(rect.width, rect.height), CV_8UC3);

    Point2f pt;
    // Get the image m_ImageTemplateFace, for alignment!
    int PixelsInFaceTemplate = 0;
    
    if(NbOfChannels == 1)
    {
        for (unsigned int Y = 0; Y < oImg.rows; Y++)
        {
            for (unsigned int X = 0; X < oImg.cols; X++)
            {
                pt.x    = X;
                pt.y    = Y;

                int k = VO_Triangle2DStructure::IsPointInTriangles(pt, triangles);
                if(k>=0)
                {
                    oImg.at<uchar>(Y, X) = (unsigned char)texture.m_MatTexture(0, PixelsInFaceTemplate);
                    PixelsInFaceTemplate++;
                }
            }
        }
    }
    else if(NbOfChannels == 3)
    {
        for (unsigned int Y = 0; Y < oImg.rows; Y++)
        {
            for (unsigned int X = 0; X < oImg.cols; X++)
            {
                pt.x    = X;
                pt.y    = Y;

                int k = VO_Triangle2DStructure::IsPointInTriangles(pt, triangles);
                if(k>=0)
                {
                    for (unsigned int chan = 0; chan < NbOfChannels; chan++)
                    {
                        oImg.at<Vec3b>(Y, X)[chan] = (unsigned char)texture.m_MatTexture(chan, PixelsInFaceTemplate);
                    }
                    PixelsInFaceTemplate++;
                }
            }
        }
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Put one texture into the template shape with a corresponding warp
 * @param       iShape          Input - shape warp from
 * @param       oShape          Input - shape warp to
 * @param       triangles       Input - contains warp information
 * @param       iImg            Input - input image, extracted texture from
 * @param       oImg            Output - output image, put the texture on to this image
 * @note        All texture vectors are of the same size. So, you can't put it into the respective triangulation
                for respective image. You've got to put a texture into a template shape first, then VO_WarpFromOneShapeToAnother
*/
unsigned int VO_TextureModel::VO_WarpFromOneShapeToAnother( const VO_Shape& iShape, 
                                                            const VO_Shape& oShape, 
                                                            const vector <VO_Triangle2DStructure>& triangles, 
                                                            const Mat& iImg, 
                                                            Mat& oImg)
{
    unsigned int NbOfChannels   = iImg.channels();
    unsigned int NbOfPoints     = iShape.GetNbOfPoints();
    unsigned int NbOfTriangles  = triangles.size();
    unsigned int NbOfPixels     = 0;
    
    Point2f src[3], dst[3];
    vector< Mat_<float> > matWarping;
    matWarping.resize(NbOfTriangles);
    vector<VO_Triangle2DStructure> warpedTriangles = oShape.GetTriangle2DStructure(triangles);
    Rect rect_in = iShape.GetShapeBoundRect();
    Rect rect_out = oShape.GetShapeBoundRect();
    oImg = Mat::zeros( Size(rect_out.width, rect_out.height), iImg.type() );
    unsigned int vIdx0, vIdx1, vIdx2;
    
    // calculate all the possible mapping (for speeding up) 95 mapping totally
    for (unsigned int i = 0; i < NbOfTriangles; i++ )
    {
        vIdx0 = triangles[i].m_vVertexIndexes[0];
        vIdx1 = triangles[i].m_vVertexIndexes[1];
        vIdx2 = triangles[i].m_vVertexIndexes[2];
        
        // Get the affine transformation for each triangle pair.
        src[0] = oShape.GetA2DPoint(vIdx0);
        src[1] = oShape.GetA2DPoint(vIdx1);
        src[2] = oShape.GetA2DPoint(vIdx2);

        dst[0] = iShape.GetA2DPoint(vIdx0);
        dst[1] = iShape.GetA2DPoint(vIdx1);
        dst[2] = iShape.GetA2DPoint(vIdx2);

        matWarping[i] = cv::getAffineTransform( src, dst );
    }
    
    float XX, YY;
    Point2f pt;
    
    if(NbOfChannels == 1)
    {
        float gray = 0.0;

        for (unsigned int Y = 0; Y < oImg.rows; Y++)
        {
            for (unsigned int X = 0; X < oImg.cols; X++)
            {
                pt.x = X + rect_out.x;
                pt.y = Y + rect_out.y;

                int k = VO_Triangle2DStructure::IsPointInTriangles(pt, warpedTriangles);
                if(k>=0)
                {
                    const Mat_<float>& sss = matWarping[k];
                    XX = sss(0,0)*pt.x + sss(0,1)*pt.y + sss(0,2);
                    YY = sss(1,0)*pt.x + sss(1,1)*pt.y + sss(1,2);
                    
                    // Since the above calculations are basically matrix calculation, 
                    // they won't be able to always ensure XX or YY are always within the image.
                    // Therefore, a constrain in the following is a must!!!
                    if (XX < 0.0f) XX = 0.0f;
                    if (YY < 0.0f) YY = 0.0f;
                    if (XX > ((float)(rect_in.width - 1) - 0.001f)) XX = (float) (rect_in.width - 1) - 0.001f;
                    if (YY > ((float)(rect_in.height - 1) - 0.001f)) YY = (float) (rect_in.height - 1) - 0.001f;

//                    // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
//                    warpsrc(0, 0) = pt.x;
//                    warpsrc(1, 0) = pt.y;
//                    warpsrc(2, 0) = 1.0;
//
//    //                cv::gemm(matWarping[k], warpsrc, 1.0, Mat(), 0.0, warpeddst);
//                    warpeddst = matWarping[k] * warpsrc;

                    // extract the B,G,R on each shape/image respectively into textures
                    // warpeddst.at<float>(0, 0) is for x while
                    // warpeddst.at<float>(1, 0) is for y. But,
                    // x determines cols, while y determines rows.
//                    VO_TextureModel::VO_CalcSubPixelTexture ( warpeddst(0, 0), warpeddst(1, 0), iImg, &gray);
                    VO_TextureModel::VO_CalcSubPixelTexture ( XX, YY, iImg, &gray);
                    oImg.at<uchar>(Y, X) = (unsigned char) gray;

                    NbOfPixels ++;
                }
            }
        }
    }
    else if(NbOfChannels == 3)
    {
        float b = 0.0, g = 0.0, r = 0.0;
        
        for (unsigned int Y = 0; Y < oImg.rows; Y++)
        {
            for (unsigned int X = 0; X < oImg.cols; X++)
            {
                pt.x = X + rect_out.x;
                pt.y = Y + rect_out.y;

                int k = VO_Triangle2DStructure::IsPointInTriangles(pt, warpedTriangles);
                if(k>=0)
                {
                    const Mat_<float>& sss = matWarping[k];
                    XX = sss(0,0)*pt.x + sss(0,1)*pt.y + sss(0,2);
                    YY = sss(1,0)*pt.x + sss(1,1)*pt.y + sss(1,2);
                    
                    // Since the above calculations are basically matrix calculation, 
                    // they won't be able to always ensure XX or YY are always within the image.
                    // Therefore, a constrain in the following is a must!!!
                    if (XX < 0.0f) XX = 0.0f;
                    if (YY < 0.0f) YY = 0.0f;
                    if (XX > ((float)(rect_in.width - 1) - 0.001f)) XX = (float) (rect_in.width - 1) - 0.001f;
                    if (YY > ((float)(rect_in.height - 1) - 0.001f)) YY = (float) (rect_in.height - 1) - 0.001f;
                    
//                    // warp from m_vTemplateTriangle2D[j] (template pixel in the specific triangle) to each shape/image
//                    warpsrc(0, 0) = pt.x;
//                    warpsrc(1, 0) = pt.y;
//                    warpsrc(2, 0) = 1.0;
//
//    //                cv::gemm(matWarping[k], warpsrc, 1.0, Mat(), 0.0, warpeddst);
//                    warpeddst = matWarping[k] * warpsrc;

                    // extract the B,G,R on each shape/image respectively into textures
                    // warpeddst.at<float>(0, 0) is for x while
                    // warpeddst.at<float>(1, 0) is for y. But,
                    // x determines cols, while y determines rows.
//                    VO_TextureModel::VO_CalcSubPixelTexture (warpeddst(0, 0), warpeddst(1, 0), iImg, &b, &g, &r);
                    VO_TextureModel::VO_CalcSubPixelTexture (XX, YY, iImg, &b, &g, &r);
                    oImg.at<Vec3b>(Y, X)[0] = (unsigned char) b;
                    oImg.at<Vec3b>(Y, X)[1] = (unsigned char) g;
                    oImg.at<Vec3b>(Y, X)[2] = (unsigned char) r;
                    
                    NbOfPixels ++;
                }
            }
        }
    }
    return NbOfPixels;
}


/**
 * @brief       image morphing
 * @param       iShape1     Input       the first shape
 * @param       iShape2     Input       the second shape
 * @param       oShapes     Output      intermediate shapes during the process of morphing
 * @param       triangles   Input       a list of triangles
 * @param       iImg1       Input       the first image
 * @param       iImg2       Input       the second image
 * @param       oImgs       Output      the output images
 * @param       step        Input       between 0 to 1, Normally, 0.1 or 0.2 are good selections
 * @note        Current version requires iShape1 and iShape2 have the same size
 **/
void VO_TextureModel::VO_Morphing(const VO_Shape& iShape1, const VO_Shape& iShape2, vector<VO_Shape>& oShapes, const vector<VO_Triangle2DStructure>& triangles, const Mat& iImg1, const Mat& iImg2, vector<Mat>& oImgs, float step)
{
    assert (iImg1.cols == iImg2.cols && iImg1.rows == iImg2.rows);
    unsigned int NbOfFrames     = cvRound(1.0/step);
    oShapes.resize(NbOfFrames+1);
    oImgs.resize(NbOfFrames+1);

    Mat morph1, morph2;

    oShapes[0]                  = iShape1;
    oShapes[NbOfFrames]         = iShape2;
    iImg1.copyTo(oImgs[0]);
    iImg2.copyTo(oImgs[NbOfFrames]);

    for(unsigned int i = 1; i < NbOfFrames; ++i)
    {
        oShapes[i] = const_cast<VO_Shape&>(iShape1)*(step*i) + const_cast<VO_Shape&>(iShape2)*(1.0-step*i);
        VO_TextureModel::VO_WarpFromOneShapeToAnother(iShape1, oShapes[i], triangles, iImg1, morph1);
        VO_TextureModel::VO_WarpFromOneShapeToAnother(iShape2, oShapes[i], triangles, iImg2, morph2);
        cv::addWeighted( morph1, step*i, morph2, 1.0-step*i, 0.0, oImgs[i] );
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Put one texture into the template shape with a corresponding warp
 * @param       iShape      Input - shape warp from
 * @param       oShape      Input - shape warp to
 * @param       triangles   Input - contains warp information
 * @param       iImg        Input - input image, extracted texture from
 * @param       oImg        Output - output image, put the texture on to this image
 * @note        All texture vectors are of the same size. So, you can't put it into the respective triangulation
                for respective image. You've got to put a texture into a template shape first, then VO_WarpFromOneShapeToAnother
*/
void VO_TextureModel::VO_PutOneTextureToOneShape(const VO_Texture& texture, const VO_Shape& oShape, const vector <VO_Triangle2DStructure>& triangles, Mat& oImg)
{
    Mat intermediateImg (oImg);
    VO_TextureModel::VO_PutOneTextureToTemplateShape(texture, triangles, intermediateImg);
    VO_Shape intermediateShape = VO_Triangle2DStructure::Triangle2D2Shape(triangles);
    VO_TextureModel::VO_WarpFromOneShapeToAnother(intermediateShape, oShape, triangles, intermediateImg, oImg);
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Calculate sub pixel texture of a point in a gray-level image
 * @param       gray            - output the gray level
 * @note        In the pictures, x represents cols, y represents rows!
 *              In Mat& image, x represents cols, y represents rows as well!!!
 * @return      void
*/
void VO_TextureModel::VO_CalcSubPixelTexture(float x, float y, const Mat& image, float* gray)
{
    assert(image.channels() == GRAYCHANNELS);
// double sec = (double)cvGetTickCount();   // Now, pretty fast already for gray-level images

    int Y0 = (int)y;
    int X0 = (int)x;
    int Y1 = (int)(y+1);
    int X1 = (int)(x+1);

    float s=x-X0;
    float t=y-Y0;
    float t1=1.0f-t;
    float s1=1.0f-s;

    float tempA = t1 * image.at<uchar>(Y0, X0)
                  + t * image.at<uchar>(Y1, X0);
    float tempB = t1 * image.at<uchar>(Y0, X1)
                  + t * image.at<uchar>(Y1, X1);
    *gray = tempA * s1 + tempB * s;

// sec = ((double)cvGetTickCount() -  sec )/  (cvGetTickFrequency());
// cout << "Interpolation time cost: " << sec << " millisec" << endl;
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Calculate sub pixel texture of a point in a color image
 * @param       gray - output the gray level
 * @note        In the pictures, x represents cols, y represents rows!
                In Mat& image, x represents cols, y represents rows as well!!!
 * @return      void
*/
void VO_TextureModel::VO_CalcSubPixelTexture(float x, float y, const Mat& image, float* b, float* g, float* r)
{
    assert(image.channels() == COLORCHANNELS);

// double sec = (double)cvGetTickCount();

    int Y0 = (int)y;
    int X0 = (int)x;
    int Y1 = (int)(y+1);
    int X1 = (int)(x+1);

    float s=x-X0;
    float t=y-Y0;
    float s1=1.0f-s;
    float t1=1.0f-t;


    float tempA_b = t1 * image.at<Vec3b>(Y0, X0)[0]
                    + t * image.at<Vec3b>(Y1, X0)[0];
    float tempB_b = t1 * image.at<Vec3b>(Y0, X1)[0]
                    + t * image.at<Vec3b>(Y1, X1)[0];
    float tempA_g = t1 * image.at<Vec3b>(Y0, X0)[1]
                    + t * image.at<Vec3b>(Y1, X0)[1];
    float tempB_g = t1 * image.at<Vec3b>(Y0, X1)[1]
                    + t * image.at<Vec3b>(Y1, X1)[1];
    float tempA_r = t1 * image.at<Vec3b>(Y0, X0)[2]
                    + t * image.at<Vec3b>(Y1, X0)[2];
    float tempB_r = t1 * image.at<Vec3b>(Y0, X1)[2]
                    + t * image.at<Vec3b>(Y1, X1)[2];
    *b = tempA_b * s1 + tempB_b * s;
    *g = tempA_g * s1 + tempB_g * s;
    *r = tempA_r * s1 + tempB_r * s;

// sec = ((double)cvGetTickCount() -  sec )/  (cvGetTickFrequency());
// cout << "Interpolation time cost: " << sec << " millisec" << endl;
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Calculate sub pixel texture of a point in image
 * @return      vector<float>   - a vector of size 1 or 3
 * @note        In the pictures, x represents cols, y represents rows!
                In Mat& image, x represents cols, y represents rows as well!!!
 * @return      void
*/
vector<float> VO_TextureModel::VO_CalcSubPixelTexture(float x, float y, const Mat& image)
{
// double sec = (double)cvGetTickCount();

    vector<float> result;
    unsigned int channels = image.channels();
    if (!(channels == 1 || channels == 3))
    {
        cerr << "Cannot deal with images with nChannels != 1 or 3." << endl;
        exit(EXIT_FAILURE);
    }

    int Y0      = (int)y;
    int X0      = (int)x;
    int Y1      = (int)(y+1);
    int X1      = (int)(x+1);

    float s     = x-X0;
    float t     = y-Y0;
    float s1    = 1.0f-s;
    float t1    = 1.0f-t;

    float LeftTopB, LeftTopG, LeftTopR;
    float LeftBottomB, LeftBottomG, LeftBottomR;
    float RightTopB, RightTopG, RightTopR;
    float RightBottomB, RightBottomG, RightBottomR;

    VO_TextureModel::VO_CalcPixelRGB(X0, Y0, image, LeftTopB, LeftTopG, LeftTopR);
    VO_TextureModel::VO_CalcPixelRGB(X0, Y1, image, LeftBottomB, LeftBottomG, LeftBottomR);
    VO_TextureModel::VO_CalcPixelRGB(X1, Y0, image, RightTopB, RightTopG, RightTopR);
    VO_TextureModel::VO_CalcPixelRGB(X1, Y1, image, RightBottomB, RightBottomG, RightBottomR);

    float tempAB, tempAG, tempAR;
    float tempBB, tempBG, tempBR;

    if(channels == 1)
    {
        tempAB = t1 * LeftTopB + t * LeftBottomB;
        tempBB = t1 * RightTopB + t * RightBottomB;
        result.push_back (tempAB * s1 + tempBB * s);
    }
    else
    {
        tempAB = t1 * LeftTopB + t * LeftBottomB;
        tempBB = t1 * RightTopB + t * RightBottomB;
        result.push_back (tempAB * s1 + tempBB * s);
        tempAG = t1 * LeftTopG + t * LeftBottomG;
        tempBG = t1 * RightTopG + t * RightBottomG;
        result.push_back (tempAG * s1 + tempBG * s);
        tempAR = t1 * LeftTopR + t * LeftBottomR;
        tempBR = t1 * RightTopR + t * RightBottomR;
        result.push_back (tempAR * s1 + tempBR * s);
    }

// sec = ((double)cvGetTickCount() -  sec )/  (cvGetTickFrequency());Vec3b
// cout << "Interpolation time cost: " << sec << " millisec" << endl;

    return result;
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Calculate RGB value for a point in image
 * @param       image
 * @return      no return value, in order to speed up !!
*/
void VO_TextureModel::VO_CalcPixelRGB(int x, int y, const Mat& image, float& B, float& G, float& R)
{
    unsigned int channels = image.channels();
    if (!(channels == 1 || channels == 3))
    {
        cerr << "Cannot deal with images with nChannels != 1 or 3." << endl;
        exit(EXIT_FAILURE);
    }

    switch(image.depth())
    {
    case IPL_DEPTH_8U:
        {
            if(channels == 1)
                B = (float)image.at<uchar>(y, x );
            else
            {
                B = (float)image.at<uchar>(y, x*3 );
                G = (float)image.at<uchar>(y, x*3+1 );
                R = (float)image.at<uchar>(y, x*3+2 );
            }
        }
        break;
    case IPL_DEPTH_8S:
        {
            if(channels == 1)
                B = (float)image.at<char>(y, x );
            else
            {
                B = (float)image.at<char>(y, x*3 );
                G = (float)image.at<char>(y, x*3+1 );
                R = (float)image.at<char>(y, x*3+2 );
            }
        }
        break;
    case IPL_DEPTH_16S:
        {
            if(channels == 1)
                B = (float)image.at<short int>(y, x );
            else
            {
                B = (float)image.at<short int>(y, x*3 );
                G = (float)image.at<short int>(y, x*3+1 );
                R = (float)image.at<short int>(y, x*3+2 );
            }
        }
        break;
     case IPL_DEPTH_16U:
        {
            if(channels == 1)
                B = (float)image.at<unsigned short int>(y, x );
            else
            {
                B = (float)image.at<unsigned short int>(y, x*3 );
                G = (float)image.at<unsigned short int>(y, x*3+1 );
                R = (float)image.at<unsigned short int>(y, x*3+2 );
            }
        }
         break;
    case IPL_DEPTH_32S:
        {
            if(channels == 1)
                B = (float)image.at<int>(y, x );
            else
            {
                B = (float)image.at<int>(y, x*3 );
                G = (float)image.at<int>(y, x*3+1 );
                R = (float)image.at<int>(y, x*3+2 );
            }
        }
        break;
    case IPL_DEPTH_32F:
        {
            if(channels == 1)
                B = image.at<float>(y, x );
            else
            {
                B = image.at<float>(y, x*3 );
                G = image.at<float>(y, x*3+1 );
                R = image.at<float>(y, x*3+2 );
            }
        }
        break;
    case IPL_DEPTH_64F:
        {
            if(channels == 1)
                B = (float)image.at<double>(y, x );
            else
            {
                B = (float)image.at<double>(y, x*3 );
                G = (float)image.at<double>(y, x*3+1 );
                R = (float)image.at<double>(y, x*3+2 );
            }
        }
        break;
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Normalized texture to reference scale texture
 * @param       inTexture       Input     - input texture
 * @param       textureSD       Input     - texture standard deviation
 * @param       outTexture      Output     - output texture in reference scale
 * @return      void
*/
void VO_TextureModel::VO_NormalizedTexture2ReferenceScale(const VO_Texture& inTexture, float textureSD, VO_Texture& outTexture)
{
    outTexture = const_cast<VO_Texture&> (inTexture) * textureSD + AVERAGEFACETEXTURE;

    outTexture.Clamp(0.0, 255.0);
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Reference scale texture back to normalized one
 * @param       inTexture       Input - reference texture
 * @param       textureSD       Input - texture standard deviation
 * @param       outTexture      Output - output normalized texture 
 * @return      void
*/
void VO_TextureModel::VO_ReferenceTextureBack2Normalize(const VO_Texture& inTexture, float textureSD, VO_Texture& outTexture)
{
    outTexture = ( const_cast<VO_Texture&> (inTexture) - AVERAGEFACETEXTURE ) / textureSD;
}


/**
 * @author        JIA Pei
 * @version       2010-02-10
 * @brief         Put edges on template face
 * @param         edges               Input     - all edges information
 * @param         templateShape       Input     - template shape
 * @param         iImg                Input     - template image
 * @param         oImg                Output     - output image with edges on oImg
 * @return        void
*/
void VO_TextureModel::VO_PutEdgesOnTemplateFace(const vector<VO_Edge>& edges, 
                                                const VO_Shape& templateShape,
                                                const Mat& iImg, 
                                                Mat& oImg)
{
    unsigned int NbOfEdges = edges.size();
    Point2f iorg,idst;

    iImg.copyTo(oImg);

    for (unsigned int i = 0; i < NbOfEdges; i++)
    {
        iorg = templateShape.GetA2DPoint(edges[i].GetIndex1() );
        idst = templateShape.GetA2DPoint(edges[i].GetIndex2() );
        cv::line( oImg, iorg, iorg, colors[8], 2, 0, 0 );
        cv::line( oImg, iorg, idst, colors[8], 1, 0, 0 );
    }
}


///**
// * @author     JIA Pei
// * @version    2010-02-10
// * @brief      Put convex hull on template face
// * @param      ch              Input - convex hull
// * @param      iImg            Input - template image
// * @param      oImg            Output - output images with convex hull on oImg
//*/
//void VO_TextureModel::VO_PutConvexHullOnTemplateFace(const Mat& iImg, Mat& oImg)
//{
//    unsigned int NbOfPointsOnConvexHull = ch.cols;
//    Point iorg,idst;
//
//    iImg.copyTo(oImg);
//
//    for (unsigned int i = 0; i < NbOfPointsOnConvexHull; i++)
//    {
//        if(i != NbOfPointsOnConvexHull - 1 )
//        {
//            iorg = cvPointFrom32f( ch.at<Point2f>(0, i ) );
//            idst = cvPointFrom32f( ch.at<Point2f>(0, i+1 ) );
//            cv::line( oImg, iorg, iorg, colors[8], 2, 0, 0 );     // emphasize the vertex
//            cv::line( oImg, iorg, idst, colors[8], 1, 0, 0 );
//        }
//        else
//        {
//            iorg = cvPointFrom32f( ch.at<Point2f>(0, i ) );
//            idst = cvPointFrom32f( ch.at<Point2f>(0, 0 ) );
//            cv::line( oImg, iorg, iorg, colors[8], 2, 0, 0 );     // emphasize the vertex
//            cv::line( oImg, iorg, idst, colors[8], 1, 0, 0 );
//        }
//    }
//}
//
//
///**
// * @author     JIA Pei
// * @version    2010-02-10
// * @brief      Put concave pixels on template face - inside convex hull but out of template face
// * @param      ch              Input - convex hull
// * @param      triangles       Input - all triangles information
// * @param      iImg            Input - template image
// * @param      oImg            Output - output images with concave pixels on oImg
//*/
//void VO_TextureModel::VO_PutConcavePixelsOnTemplateFace(const Mat& ch, const vector <VO_Triangle2DStructure>& triangles,
//                                               const Mat& iImg, Mat& oImg)
//{
//    iImg.copyTo(oImg);
//
//    Point2f pt;
//    for (unsigned int i = 0; i < iImg.rows; i++)
//    {
//        for (unsigned int j = 0; j < iImg.cols; j++)
//        {
//            pt.x = (float)j;
//            pt.y = (float)i;
//
//            if( VO_TextureModel::VO_IsPointInConvexHull(pt, ch, true) && !VO_TextureModel::VO_IsPointInTemplateFace(triangles, pt) )
//            {
//                Point iorg = cvPointFrom32f(pt);
//                cv::line( oImg, iorg, iorg, colors[8], 1, 0, 0 );   // Here, must not emphasize the points
//            }
//        }
//    }
//}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Put every single triangle onto template face
 * @param       triangles       Input - all triangles information
 * @param       iImg            Input - template image
 * @param       oImgs           Output - output images with single triangle on every single oImg
 * @return      void
*/
void VO_TextureModel::VO_PutTrianglesOnTemplateFace(const vector <VO_Triangle2DStructure>& triangles,
                                                    const Mat& iImg, 
                                                    vector<Mat>& oImgs)
{
    unsigned int NbOfTriangles = triangles.size();
    oImgs.resize(NbOfTriangles);

    vector<Point2f> iorgFloat, idstFloat;
    iorgFloat.resize(3);
    idstFloat.resize(3);
        
    for(unsigned int i = 0; i < NbOfTriangles; i++)
    {
        iImg.copyTo(oImgs[i]);

        iorgFloat[0]    = triangles[i].GetA2DPoint(0);
        iorgFloat[1]    = triangles[i].GetA2DPoint(1);
        iorgFloat[2]    = triangles[i].GetA2DPoint(2);
        idstFloat[0]    = triangles[i].GetA2DPoint(1);
        idstFloat[1]    = triangles[i].GetA2DPoint(2);
        idstFloat[2]    = triangles[i].GetA2DPoint(0);

        cv::line( oImgs[i], iorgFloat[0], idstFloat[0], colors[8], 1, 0, 0 );
        cv::line( oImgs[i], iorgFloat[1], idstFloat[1], colors[8], 1, 0, 0 );
        cv::line( oImgs[i], iorgFloat[2], idstFloat[2], colors[8], 1, 0, 0 );
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       draw all PDM ellipses onto the textured template face
 * @param       iShape      Input   -    the input shape
 * @param       iImg        Input   -    the input image
 * @param       oImg        Output  -    the output image
 * @return      void
 */
void VO_TextureModel::VO_PutPDMEllipsesOnTemplateFace(const vector<VO_Ellipse>& ellipses, const Mat& iImg, Mat& oImg)
{
    unsigned int NbOfEllipses =  ellipses.size();
    iImg.copyTo(oImg);
    Point2f iPoint;

    for(unsigned int i = 0; i < NbOfEllipses; i++)
    {
        Point2f pt = ellipses.at(i).GetCOG();

        cv::ellipse(oImg, Point( (int)pt.x, (int)pt.y ),
                    Size(ellipses.at(i).GetAxisXHalfLen(), ellipses.at(i).GetAxisYHalfLen()),
                    ellipses.at(i).GetAngle(),
                    ellipses.at(i).GetStartAngle(),
                    ellipses.at(i).GetEndAngle(),
                    colors[7],
                    1,
                    0,
                    0);
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       draw the shape's key points onto the textured template face
 * @param       iShape      Input    -    the input shape
 * @param       iImg        Input    -     the input image
 * @param       oImg        Output   -    the output image
 * @return      void
 */
void VO_TextureModel::VO_PutShapeOnTemplateFace(const VO_Shape& iShape, const Mat& iImg, Mat& oImg)
{
    unsigned int NbOfPoints =  iShape.GetNbOfPoints();
    iImg.copyTo(oImg);
    Point2f iPoint;

    for(unsigned int i = 0; i < NbOfPoints; i++)
    {
        iPoint    = iShape.GetA2DPoint(i);

        cv::line( oImg, iPoint, iPoint, colors[7], 2, 0, 0 );
    }
}


/**
 * @author      JIA Pei
 * @version     2010-04-07
 * @brief       texture parameters constrain
 * @param       ioT         Input and Output - texture parameters
 * @param       nSigma      Input - number of sigmas
 * @return      void
*/
void VO_TextureModel::VO_TextureParameterConstraint(Mat_<float>& ioT, float nSigma)
{
    for (unsigned int i = 0; i < ioT.cols; ++i)
    {
        float ct = nSigma * sqrt(this->m_PCANormalizedTexture.eigenvalues.at<float>(i,0) );
        if ( ioT(0, i) > ct )
        {
            ioT(0, i) = ct;
        }
        else if ( ioT(0, i) < -ct )
        {
            ioT(0, i) = -ct;
        }
    }
}


/**
* @author     JIA Pei
* @version    2010-04-10
* @brief      texture parameters back project to normalized texture
* @param      iTexture        Input - input normalized texture
* @param      outT            Output - the projected texture parameters
*/
void VO_TextureModel::VO_NormalizedTextureProjectToTParam(const VO_Texture& iTexture, Mat_<float>& outT) const
{
    this->m_PCANormalizedTexture.project(iTexture.GetTheTextureInARow(), outT);
}


/**
* @author      JIA Pei
* @version     2010-02-10
* @brief       texture parameters back project to normalized texture
* @param       inC          Input - input texture parameters
* @param       oTexture     Output - the back projected texture in a row
* @param       tr           Input - Number of texture representations
*/
void VO_TextureModel::VO_TParamBackProjectToNormalizedTexture(const Mat_<float>& inT, VO_Texture& oTexture, int tr) const
{
    oTexture.SetTheTexture(this->m_PCANormalizedTexture.backProject(inT), tr);
}

/**
 * @author         JIA Pei
 * @version        2010-02-10
 * @brief          texture parameters back project to normalized texture
 * @param          inC             Input - input texture parameters
 * @param          oTexture        Output - the back projected texture in a row
*/
void VO_TextureModel::VO_TParamBackProjectToNormalizedTexture(const Mat_<float>& inT, Mat_<float>& oTextureMat) const
{
    oTextureMat = this->m_PCANormalizedTexture.backProject(inT);
}

/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Calculate all parameters for an arbitrary texture
 * @param       iTexture        Input   - input texture
 * @param       oTexture        Output  - output texture
 * @param       outT            Output  - texture parameters
 * @return      void
*/
void VO_TextureModel::VO_CalcAllParams4AnyTexture(const Mat_<float>& iTexture, Mat_<float>& oTexture, Mat_<float>& outT)
{
    // Here, for VO_Texture; there is no point to know how many texture representations for each VO_Texture
    VO_Texture oT(iTexture);
    oT.Normalize();
    oTexture = oT.GetTheTextureInARow();
    this->m_PCANormalizedTexture.project(oTexture, outT );
}


/**
 * @author      JIA Pei
 * @version     2010-02-10
 * @brief       Calculate all parameters for an arbitrary texture
 * @param       ioTexture       Input and Output    - input texture, output normalized texture
 * @param       outT            Output              - output texture model parameters
 * @return      void
*/
void VO_TextureModel::VO_CalcAllParams4AnyTexture(VO_Texture& ioTexture, Mat_<float>& outT)
{
    ioTexture.Normalize();
    this->m_PCANormalizedTexture.project(ioTexture.GetTheTextureInARow(), outT );
}


/**
 * @author      JIA Pei
 * @version     2010-02-05
 * @brief       Load Training data for texture model
 * @return      void
*/
bool VO_TextureModel::VO_LoadTextureTrainingData(   const vector<string>& allImgFiles4Training,
                                                    unsigned int channels,
                                                    int trm )
{
    this->m_vStringTrainingImageNames       = allImgFiles4Training;
    this->m_iNbOfChannels                   = channels;
    this->m_iTextureRepresentationMethod    = trm;
    
    this->m_vTextures.resize(this->m_iNbOfSamples);
    this->m_vNormalizedTextures.resize(this->m_iNbOfSamples);
    Mat img;
    
    for(unsigned int i = 0; i < this->m_iNbOfSamples; ++i)
    {
        if(this->m_iNbOfChannels == 1)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 0 );
        else if (this->m_iNbOfChannels == 3)
            img = imread ( this->m_vStringTrainingImageNames[i].c_str (), 1 );
        else
            cerr << "We can't deal with image channels not equal to 1 or 3!" << endl;

        double start = (double)cvGetTickCount();
        // Explained by JIA Pei -- warping
        if ( !VO_TextureModel::VO_LoadOneTextureFromShape(  this->m_vShapes[i], 
                                                            img, 
                                                            this->m_vTemplateTriangle2D, 
                                                            this->m_vTemplatePointWarpInfo,
                                                            this->m_vTextures[i], 
                                                            this->m_iTextureRepresentationMethod) )
        {
            cout << "Texture Fail to Load at image " << i << endl;
            return false;
        }

        double end = (double)cvGetTickCount();
        double elapsed = (end - start) / (cvGetTickFrequency()*1000.0);
    }

    return true;
}


/**
 * @author      JIA Pei
 * @version     2010-02-05
 * @brief       build Texture Model
 * @param       allLandmarkFiles4Training   Input - all training landmark files
 * @param       allImgFiles4Training        Input - all training image files
 * @param       shapeinfoFileName           Input - shape info file
 * @param       database                    Input - which database is it?
 * @param       channels                    Input - How many channels are to be used?
 * @param       trm                         Input - texture representation method
 * @param       TPShape                     Input - truncated percentage for shape model
 * @param       TPTexture                   Input - truncated percentage for texture model
 * @param       useKnownTriangles           Input - use known triangle structures??
 * @note        Refer to "Statistical Models of Appearance for Computer Vision" page 31, Cootes
 * @return      void
*/
void VO_TextureModel::VO_BuildTextureModel( const vector<string>& allLandmarkFiles4Training,
                                            const vector<string>& allImgFiles4Training,
                                            const string& shapeinfoFileName, 
                                            unsigned int database,
                                            unsigned int channels,
                                            int trm, 
                                            float TPShape, 
                                            float TPTexture, 
                                            bool useKnownTriangles)
{
    if (allLandmarkFiles4Training.size() != allImgFiles4Training.size() )
        cerr << "allLandmarkFiles4Training should have the same number of allImgFiles4Training! " << endl;

    this->VO_BuildShapeModel(allLandmarkFiles4Training, shapeinfoFileName, database, TPShape, useKnownTriangles);
    this->m_iNbOfPixels                         = VO_TextureModel::VO_CalcPointWarpingInfo(this->m_vTemplateTriangle2D, this->m_vTemplatePointWarpInfo);
    this->VO_LoadTextureTrainingData( allImgFiles4Training, channels, trm);

    this->m_iNbOfTextureRepresentations         = this->m_vTextures[0].GetNbOfTextureRepresentation();
    this->m_iNbOfTextures                         = this->m_iNbOfPixels*this->m_iNbOfTextureRepresentations;
    this->m_iNbOfEigenTexturesAtMost             = MIN(this->m_iNbOfSamples, this->m_iNbOfTextures);
    this->m_fTruncatedPercent_Texture           = TPTexture;
    Mat_<float> matNormalizedTextures            = Mat_<float>::zeros(this->m_iNbOfSamples, this->m_iNbOfTextures);
    Mat_<float> matNormalizedMeanTextures        = Mat_<float>::zeros(1, m_iNbOfTextures);

    // Normalize all textures
    this->m_fAverageTextureStandardDeviation     = VO_TextureModel::VO_NormalizeAllTextures(this->m_vTextures, this->m_vNormalizedTextures);
    VO_TextureModel::VO_CalcMeanTexture(this->m_vNormalizedTextures, this->m_VONormalizedMeanTexture);
    // Calculate reference texture
    VO_TextureModel::VO_NormalizedTexture2ReferenceScale(this->m_VONormalizedMeanTexture, this->m_fAverageTextureStandardDeviation, this->m_VOReferenceTexture);
//VO_TextureModel::VO_PutOneTextureToTemplateShape(this->m_VOReferenceTexture, this->m_vTemplateTriangle2D, this->m_ImageTemplateFace);
//imwrite("template.jpg",this->m_ImageTemplateFace);

    matNormalizedMeanTextures = this->m_VONormalizedMeanTexture.GetTheTextureInARow();
    for(unsigned int i = 0; i < this->m_iNbOfSamples; ++i)
    {
        Mat_<float> tmpRow = matNormalizedTextures.row(i);
        this->m_vNormalizedTextures[i].GetTheTextureInARow().copyTo(tmpRow);
    }
    this->m_PCANormalizedTexture(matNormalizedTextures, matNormalizedMeanTextures, CV_PCA_DATA_AS_ROW, this->m_iNbOfEigenTexturesAtMost );
    // to decide how many components to be selected
    this->m_iNbOfTextureEigens = 0;

    double SumOfEigenValues = cv::sum( this->m_PCANormalizedTexture.eigenvalues ).val[0];
    double ps = 0.0f;

    for(unsigned int i = 0; i < this->m_iNbOfEigenTexturesAtMost; i++)
    {
        ps += this->m_PCANormalizedTexture.eigenvalues.at<float>(i,0 );
        ++this->m_iNbOfTextureEigens;
        if( ps/SumOfEigenValues >= this->m_fTruncatedPercent_Texture) break;
    }
    // m_iNbOfTextureEigens decided. For simplicity, we carry out PCA once again.
    this->m_PCANormalizedTexture(matNormalizedTextures, matNormalizedMeanTextures, CV_PCA_DATA_AS_ROW, this->m_iNbOfTextureEigens );

    //////////////////////////////////////////////////////////////////////////
    // Calculate m_vNormalizedPointWarpInfo
    //////////////////////////////////////////////////////////////////////////
    Point2f src[3];
    Point2f dst[3];

    // warp from reference triangles to normalized triangles, it's basically just a translation and scaling
    vector< Mat_<float> > matWarping;
    matWarping.resize(this->m_iNbOfTriangles);

    // calculate all the possible mapping (for speeding up) 95 mapping totally
    // Here, actually, a translation and a scaling can do this as well!
    for (unsigned int k = 0; k < this->m_iNbOfTriangles; k++ )
    {
        // Get the affine transformation for each triangle pair.
        src[0] = this->m_vTemplateTriangle2D[k].GetA2DPoint(0);
        src[1] = this->m_vTemplateTriangle2D[k].GetA2DPoint(1);
        src[2] = this->m_vTemplateTriangle2D[k].GetA2DPoint(2);

        dst[0] = this->m_VOAlignedMeanShape.GetA2DPoint( this->m_vTemplateTriangle2D[k].GetVertexIndex(0) );
        dst[1] = this->m_VOAlignedMeanShape.GetA2DPoint( this->m_vTemplateTriangle2D[k].GetVertexIndex(1) );
        dst[2] = this->m_VOAlignedMeanShape.GetA2DPoint( this->m_vTemplateTriangle2D[k].GetVertexIndex(2) );

        matWarping[k] = cv::getAffineTransform( src, dst );
    }

    unsigned int triangleIndex;
    Point2f pt, pt0;
    Mat_<float> warpsrc     = Mat_<float>::ones(3, 1);
    Mat_<float> warpeddst     = Mat_<float>::zeros(2, 1);
    for (unsigned int i = 0; i < this->m_iNbOfPixels; i++)
    {
        // JIA Pei. 2006-11-25. You will see the following (int) is very important
        // without (int), the result here is not correct at all!!
        pt = this->m_vTemplatePointWarpInfo[i].GetPosition();
        triangleIndex = this->m_vTemplatePointWarpInfo[i].GetTriangleIndex();

        warpsrc(0, 0) = pt.x;
        warpsrc(1, 0) = pt.y;
        warpsrc(2, 0) = 1.0;

        warpeddst = matWarping[triangleIndex] * warpsrc;
        pt0.x = warpeddst.at<float>(0, 0);
        pt0.y = warpeddst.at<float>(1, 0);

        VO_WarpingPoint tempNormalizedPixelTriangle;

        tempNormalizedPixelTriangle.SetPosition(pt0);
        tempNormalizedPixelTriangle.SetTriangleIndex(triangleIndex);
        tempNormalizedPixelTriangle.SetPointIndex(i);
        tempNormalizedPixelTriangle.SetTriangle2DStructure(this->m_vNormalizedTriangle2D[triangleIndex] );

        // Very important!! Note by JIA Pei, push_back cannot perform on vector< vector < > >
        this->m_vNormalizedPointWarpInfo.push_back (tempNormalizedPixelTriangle);
    }
    //////////////////////////////////////////////////////////////////////////

    // Calculate template images
    VO_TextureModel::VO_PutOneTextureToTemplateShape(this->m_VOReferenceTexture, this->m_vTemplateTriangle2D, this->m_ImageTemplateFace);
    VO_TextureModel::VO_PutEdgesOnTemplateFace(this->m_vEdge, this->m_VOReferenceShape, this->m_ImageTemplateFace, this->m_ImageEdges);

    //////////////////////////////////////////////////////////////////////////
    VO_Shape tmpRefShape = this->m_VOAlignedMeanShape*this->m_fAverageShapeSize;
    vector<VO_Ellipse> refEllipses = this->m_VOPDM.GetPDMEllipses();
    refEllipses.resize(this->m_iNbOfPoints);
    for(unsigned int i = 0; i < this->m_iNbOfPoints; i++)
    {
        refEllipses[i].ScaleCenter(this->m_fAverageShapeSize);
        refEllipses[i] *= this->m_fAverageShapeSize;
        //refEllipses[i] *= this->m_fAverageShapeSize*3.0;
//        Rect rect = refEllipses[i].CalcBoundingRect();
//        Mat tmpImg= Mat::zeros( rect.height, rect.width, this->m_ImageTemplateFace.type());
//        Mat_<float> ellipseMin = Mat_<float>::zeros(2, 1);
//        ellipseMin(0,0) = rect.x;
//        ellipseMin(1,0) = rect.y;
//        refEllipses[i].Translate(-ellipseMin);
//
//        Point2f pt = refEllipses[i].GetCOG();
//        cv::ellipse(tmpImg, Point( (int)pt.x, (int)pt.y ),
//                    Size(refEllipses[i].GetAxisXHalfLen(), refEllipses[i].GetAxisYHalfLen()),
//                    refEllipses[i].GetAngle(),
//                    refEllipses[i].GetStartAngle(),
//                    refEllipses[i].GetEndAngle(),
//                    colors[3],
//                    1,
//                    0,
//                    0);
//        imwrite("tmp.jpg", tmpImg);
    }
    Rect brect = VO_Ellipse::VO_CalcBoundingRect4MultipleEllipses(refEllipses);
    Mat_<float> ellipseMin = Mat_<float>::zeros(2, 1);
    ellipseMin(0,0) = brect.x;
    ellipseMin(1,0) = brect.y;
    this->m_ImageEllipses = Mat::zeros(brect.height, brect.width, this->m_ImageTemplateFace.type());
    tmpRefShape.Translate( -ellipseMin );
    for(unsigned int i = 0; i < this->m_iNbOfPoints; i++)
        refEllipses[i].Translate(-ellipseMin);
    VO_TextureModel::VO_PutShapeOnTemplateFace(tmpRefShape, this->m_ImageEllipses, this->m_ImageEllipses);
    //    imwrite("edges.jpg", this->m_ImageEllipses);
    VO_TextureModel::VO_PutPDMEllipsesOnTemplateFace(refEllipses, this->m_ImageEllipses, this->m_ImageEllipses);
    
    //////////////////////////////////////////////////////////////////////////
}


/**
 * @author      JIA Pei
 * @version     2010-02-13
 * @brief       Save AAM to a specified folder
 * @param       fd      Input - the folder that AAM to be saved to
 * @return      void
*/
void VO_TextureModel ::VO_Save(const string& fd)
{
    VO_ShapeModel::VO_Save(fd);

    string fn = fd+"/TextureModel";
    if (!boost::filesystem::is_directory(fn) )
        boost::filesystem::create_directory( fn );

    fstream fp;
    string tempfn;

    // TextureModel
    tempfn = fn + "/TextureModel" + ".txt";
    fp.open(tempfn.c_str (), ios::out);

    fp << "m_iTextureRepresentationMethod" << endl << this->m_iTextureRepresentationMethod << endl;         // m_iTextureRepresentationMethod
    fp << "m_iNbOfTextureRepresentations" << endl << this->m_iNbOfTextureRepresentations << endl;             // m_iNbOfTextureRepresentations
    fp << "m_iNbOfChannels" << endl << this->m_iNbOfChannels << endl;                                        // m_iNbOfChannels
    fp << "m_iNbOfPixels" << endl << this->m_iNbOfPixels << endl;                                           // m_iNbOfPixels
    fp << "m_iNbOfTextures" << endl << this->m_iNbOfTextures << endl;                                       // m_iNbOfTextures
    fp << "m_iNbOfEigenTexturesAtMost" << endl << this->m_iNbOfEigenTexturesAtMost << endl;                 // m_iNbOfEigenTexturesAtMost
    fp << "m_iNbOfTextureEigens" << endl << this->m_iNbOfTextureEigens << endl;                             // m_iNbOfTextureEigens
    fp << "m_fAverageTextureStandardDeviation" << endl << this->m_fAverageTextureStandardDeviation << endl; // m_fAverageTextureStandardDeviation
    fp << "m_fTruncatedPercent_Texture" << endl << this->m_fTruncatedPercent_Texture << endl;               // m_fTruncatedPercent_Texture
    fp.close();fp.clear();

    // m_PCANormalizedTextureMean
    tempfn = fn + "/m_PCANormalizedTextureMean" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_PCANormalizedTextureMean" << endl;
    fp << Mat_<float>(this->m_PCANormalizedTexture.mean);
    fp.close();fp.clear();

    // m_PCANormalizedTextureEigenValues
    tempfn = fn + "/m_PCANormalizedTextureEigenValues" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_PCANormalizedTextureEigenValues" << endl;
    fp << Mat_<float>(this->m_PCANormalizedTexture.eigenvalues);
    fp.close();fp.clear();
    
    // m_PCANormalizedTextureEigenVectors
    tempfn = fn + "/m_PCANormalizedTextureEigenVectors" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_PCANormalizedTextureEigenVectors" << endl;
    fp << Mat_<float>(this->m_PCANormalizedTexture.eigenvectors);
    fp.close();fp.clear();

    // m_VONormalizedMeanTexture
    tempfn = fn + "/m_VONormalizedMeanTexture" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_VONormalizedMeanTexture" << endl;
    fp << this->m_VONormalizedMeanTexture;
    fp.close();fp.clear();
    
    // m_VOReferenceTexture
    tempfn = fn + "/m_VOReferenceTexture" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_VOReferenceTexture" << endl;
    fp << this->m_VOReferenceTexture;
    fp.close();fp.clear();

    // m_vTextures
    tempfn = fn + "/m_vTextures" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_vTextures" << endl;
    fp << this->m_vTextures;
    fp.close();fp.clear();

    // m_vNormalizedTextures
    tempfn = fn + "/m_vNormalizedTextures" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_vNormalizedTextures" << endl;
    fp << this->m_vNormalizedTextures;
    fp.close();fp.clear();
    
    // m_vTemplatePointWarpInfo
    tempfn = fn + "/m_vTemplatePointWarpInfo" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_vTemplatePointWarpInfo" << endl;
    fp << this->m_vTemplatePointWarpInfo;
    fp.close();fp.clear();

    // m_vNormalizedPointWarpInfo
    tempfn = fn + "/m_vNormalizedPointWarpInfo" + ".txt";
    fp.open(tempfn.c_str (), ios::out);
    fp << "m_vNormalizedPointWarpInfo" << endl;
    fp << this->m_vNormalizedPointWarpInfo;
    fp.close();fp.clear();

    /** Template face image */
    tempfn = fn + "/Reference.jpg";
    imwrite(tempfn.c_str(), this->m_ImageTemplateFace);

    /** Image of edges */
    tempfn = fn + "/edges.jpg";
    imwrite(tempfn.c_str(), this->m_ImageEdges);

    /** Image of ellipse */
    tempfn = fn + "/ellipses.jpg";
    imwrite(tempfn.c_str(), this->m_ImageEllipses);

    /** How many triangles totally */
    vector<Mat> imageTriangles;
    VO_TextureModel::VO_PutTrianglesOnTemplateFace(this->m_vTemplateTriangle2D, this->m_ImageTemplateFace, imageTriangles);
    string trianglestr;
    stringstream ssi;
    string stri;
    for (unsigned int i = 0; i < imageTriangles.size(); i++)
    {
        ssi << i;
        ssi >> stri;
        if(stri.length() == 2)
            trianglestr = fn + "/triangle0" + stri + ".jpg";
        else if(stri.length() == 1)
            trianglestr = fn + "/triangle00" + stri + ".jpg";
        else
            trianglestr = fn + "/triangle" + stri + ".jpg";

        imwrite(trianglestr.c_str(), imageTriangles[i] );

        ssi.clear();
    }
}


/**
 * @author      JIA Pei
 * @version     2010-02-13
 * @brief       Load all Texture Modeldata from a specified folder
 * @param       fd      Input - the folder that Texture Model to be loaded from
 * @return      void
*/
void VO_TextureModel ::VO_Load(const string& fd)
{
    VO_ShapeModel::VO_Load(fd);
    
    this->VO_LoadParameters4Fitting(fd);
    
    string fn = fd+"/TextureModel";
    if (!boost::filesystem::is_directory(fn) )
    {
        cout << "TextureModel subfolder is not existing. " << endl;
        exit(EXIT_FAILURE);
    }

    ifstream fp;
    string tempfn;
    string temp;
    
    // m_vTextures
    tempfn = fn + "/m_vTextures" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    this->m_vTextures.resize(this->m_iNbOfSamples);
    fp >> this->m_vTextures;
    fp.close();fp.clear();

    // m_vNormalizedTextures
    tempfn = fn + "/m_vNormalizedTextures" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    this->m_vNormalizedTextures.resize(this->m_iNbOfSamples);
    fp >> this->m_vNormalizedTextures;
    fp.close();fp.clear();
    
    /** Image of edges */
    tempfn = fn + "/edges.jpg";
    this->m_ImageEdges = imread(tempfn.c_str(), CV_LOAD_IMAGE_ANYCOLOR );

}


/**
 * @author      JIA Pei
 * @version     2010-02-13
 * @brief       Load all AAM data from a specified folder for later fitting
 * @param       fd      Input - the folder that AAM to be loaded from
 * @return      void
*/
void VO_TextureModel::VO_LoadParameters4Fitting(const string& fd)
{
    VO_ShapeModel::VO_LoadParameters4Fitting(fd);

    string fn = fd+"/TextureModel";
    if (!boost::filesystem::is_directory(fn) )
    {
        cout << "TextureModel subfolder is not existing. " << endl;
        exit(EXIT_FAILURE);
    }

    ifstream fp;
    string tempfn;
    string temp;

    // TextureModel
    tempfn = fn + "/TextureModel" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp >> this->m_iTextureRepresentationMethod;
    fp >> temp >> this->m_iNbOfTextureRepresentations;
    fp >> temp >> this->m_iNbOfChannels;
    fp >> temp >> this->m_iNbOfPixels;
    fp >> temp >> this->m_iNbOfTextures;
    fp >> temp >> this->m_iNbOfEigenTexturesAtMost;
    fp >> temp >> this->m_iNbOfTextureEigens;
    fp >> temp >> this->m_fAverageTextureStandardDeviation;
    fp >> temp >> this->m_fTruncatedPercent_Texture;
    fp.close();fp.clear();
    
    this->m_PCANormalizedTexture = cv::PCA();

    // m_PCANormalizedTextureMean
    this->m_PCANormalizedTexture.mean = Mat_<float>::zeros(1, this->m_iNbOfTextures);
    tempfn = fn + "/m_PCANormalizedTextureMean" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_PCANormalizedTexture.mean;
    fp.close();fp.clear();
    
    // m_PCANormalizedTextureEigenValues
    this->m_PCANormalizedTexture.eigenvalues = Mat_<float>::zeros(this->m_iNbOfTextureEigens, 1);
    tempfn = fn + "/m_PCANormalizedTextureEigenValues" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_PCANormalizedTexture.eigenvalues;
    fp.close();fp.clear();
    
    // m_PCANormalizedTextureEigenVectors
    this->m_PCANormalizedTexture.eigenvectors = Mat_<float>::zeros(this->m_iNbOfTextureEigens, this->m_iNbOfTextures);
    tempfn = fn + "/m_PCANormalizedTextureEigenVectors" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_PCANormalizedTexture.eigenvectors;
    fp.close();fp.clear();

    // m_VONormalizedMeanTexture
    this->m_VONormalizedMeanTexture.m_MatTexture = Mat_<float>::zeros(this->m_iNbOfTextureRepresentations, this->m_iNbOfPixels);
    tempfn = fn + "/m_VONormalizedMeanTexture" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_VONormalizedMeanTexture;
    fp.close();fp.clear();

    // m_VOReferenceTexture
    this->m_VOReferenceTexture.m_MatTexture = Mat_<float>::zeros(this->m_iNbOfTextureRepresentations, this->m_iNbOfPixels);
    tempfn = fn + "/m_VOReferenceTexture" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    fp >> this->m_VOReferenceTexture;
    fp.close();fp.clear();

    // m_vTemplatePointWarpInfo
    tempfn = fn + "/m_vTemplatePointWarpInfo" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    this->m_vTemplatePointWarpInfo.resize(this->m_iNbOfPixels);
    fp >> this->m_vTemplatePointWarpInfo;
    fp.close();fp.clear();
    for (unsigned int i = 0; i < this->m_iNbOfPixels; i++)
    {
        this->m_vTemplatePointWarpInfo[i].SetTriangle2DStructure( this->m_vTemplateTriangle2D[this->m_vTemplatePointWarpInfo[i].GetTriangleIndex ()] );
    }

    // m_vNormalizedPointWarpInfo
    tempfn = fn + "/m_vNormalizedPointWarpInfo" + ".txt";
    fp.open(tempfn.c_str (), ios::in);
    fp >> temp;
    this->m_vNormalizedPointWarpInfo.resize(this->m_iNbOfPixels);
    fp >> this->m_vNormalizedPointWarpInfo;
    fp.close();fp.clear();
    for (unsigned int i = 0; i < this->m_iNbOfPixels; i++)
    {
        this->m_vNormalizedPointWarpInfo[i].SetTriangle2DStructure( this->m_vNormalizedTriangle2D[this->m_vNormalizedPointWarpInfo[i].GetTriangleIndex ()] );
    }

    /** Template face image */
    tempfn = fn + "/Reference.jpg";
    this->m_ImageTemplateFace = imread(tempfn.c_str(), CV_LOAD_IMAGE_ANYCOLOR );
}

