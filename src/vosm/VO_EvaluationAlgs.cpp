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
* Create Date:      2010-11-04                                              *
* Revise Date:      2012-03-22                                              *
*****************************************************************************/


#include "VO_EvaluationAlgs.h"
#include "VO_CVCommon.h"

/**
* @brief    to judge whether point "pt" is within rectangle "rect"
* @param    pt      - input    the concerned point
* @param    rect    - input    the concerned rectangle
*/
bool CEvaluationAlgs::IsPointInRect(const Point2d& pt, const Rect& rect)
{
    if(pt.x >= rect.x && pt.x < rect.x+rect.width &&
        pt.y >= rect.y && pt.y < rect.y+rect.height)
        return true;
    else
        return false;
}

/**
* @brief    calculate Overlapped Area Ratio and Center Offset
* @param    dObjRect    - input, detected object rectangle
                        rectangle that encompasses the detected object
* @param    lObjRect    - input, located object rectangle
                        rectangle loaded from annotated object (realistic object)
* @param    oar     - output    overlapped area ratio, 
                    overlapped area / entire area covered by two rectangles
* @param    co      - output    center offset
                    the distance between two centers of dObjRect and lObjRect
*/
void CEvaluationAlgs::StandardDetectionEvaluation(  const Rect& dObjRect,
                                                    const Rect& lObjRect,
                                                    double& oar,
                                                    double& co)
{
    unsigned int LeftMost, TopMost, RightMost, BottomMost;
    LeftMost = (dObjRect.x < lObjRect.x) ? dObjRect.x : lObjRect.x;
    TopMost = (dObjRect.y < lObjRect.y) ? dObjRect.y : lObjRect.y;
    RightMost = (dObjRect.x + dObjRect.width > lObjRect.x + lObjRect.width)
        ? dObjRect.x + dObjRect.width : lObjRect.x + lObjRect.width;
    BottomMost = (dObjRect.y + dObjRect.height > lObjRect.y + lObjRect.height)
        ? dObjRect.y + dObjRect.height : lObjRect.y + lObjRect.height;

    unsigned int overlappedarea = 0;
    unsigned int entirearea = 0;
    Point2d pt;
    bool isInFirst, isInSecond;
    for(unsigned int i = LeftMost; i < RightMost; ++i)
    {
        for(unsigned int j = TopMost; j < BottomMost; ++j)
        {
            pt.x = i;
            pt.y = j;
            if(CEvaluationAlgs::IsPointInRect(pt, dObjRect) )
                isInFirst = true;
            else
                isInFirst =  false;
            if(CEvaluationAlgs::IsPointInRect(pt, lObjRect) )
                isInSecond = true;
            else
                isInSecond = false;
            if(isInFirst || isInSecond)
                entirearea++;
            if(isInFirst && isInSecond)
                overlappedarea++;
        }
    }
    oar = (double)overlappedarea / (double)entirearea;

    double dxCenter = (double)dObjRect.x + (double)dObjRect.width/2.0;
    double dyCenter = (double)dObjRect.y + (double)dObjRect.height/2.0;
    double lxCenter = (double)lObjRect.x + (double)lObjRect.width/2.0;
    double lyCenter = (double)lObjRect.y + (double)lObjRect.height/2.0;
    co  = sqrt ( pow( (dxCenter-lxCenter), 2.0)
                + pow( (dyCenter-lyCenter), 2.0) );
}


/**
* @param    face        - input     detected face rectangle
* @param    lEyeCent    - output    center of the left eye
* @param    rEyeCent    - output    center of the right eye
*/
void CEvaluationAlgs::PridictEyeCenters(const Rect& face,
                                        Point2d& lEyeCent,
                                        Point2d& rEyeCent)
{
    lEyeCent.x  = (double)face.width/20.0 + 9.0*(double)face.width/20.0*0.5+face.x;
    lEyeCent.y  = (double)face.height/10.0 + (double)face.height/3.0*0.5+face.y;
    rEyeCent.x  = (double)face.width/2.0 + 9.0*(double)face.width/20.0*0.5+face.x;
    rEyeCent.y  = (double)face.height/10.0 + (double)face.height/3.0*0.5+face.y;
}


/**
* @brief    Evaluation for all image files from an image sequence.
* @param    detectedFaces   - input a vector of detected faces
* @param    lEyeCents       - input a vector of detected left eye centers
* @param    rEyeCents       - input a vector of detected right eye centers
* @return   a vector of dEyes    -- refer to Cristinacce paper
*/
vector<double> CEvaluationAlgs::CristinacceDEyes(
    const vector<Rect>& detectedFaces,
    const vector<Point2d>& lEyeCents,
    const vector<Point2d>& rEyeCents )
{
    assert(detectedFaces.size() == lEyeCents.size() );
    assert(detectedFaces.size() == rEyeCents.size() );

    Point2d predictedLEyeCent, predictedREyeCent;
    unsigned int NbOfSamples = detectedFaces.size();
    vector<double> dEyes(NbOfSamples, 0.0);
    double dLeft, dRight, dLeftRight;
    for (unsigned int i = 0; i < NbOfSamples; i++)
    {
        // if face hasn't been detected
        if(detectedFaces[i].x < 0 && detectedFaces[i].y < 0 &&
            detectedFaces[i].width < 0 && detectedFaces[i].height < 0)
        {
            dEyes[i] = DBL_MAX;
        }
        else
        {
            CEvaluationAlgs::PridictEyeCenters( detectedFaces[i],
                                                predictedLEyeCent,
                                                predictedREyeCent);
            dLeft = sqrt ( pow( (predictedLEyeCent.x - lEyeCents[i].x), 2.0)
                + pow( (predictedLEyeCent.y - lEyeCents[i].y), 2.0) );
            dRight = sqrt ( pow( (predictedREyeCent.x - rEyeCents[i].x), 2.0)
                + pow( (predictedREyeCent.y - rEyeCents[i].y), 2.0) );
            dLeftRight = sqrt ( pow( (lEyeCents[i].x - rEyeCents[i].x), 2.0)
                + pow( (lEyeCents[i].y - rEyeCents[i].y), 2.0) );
            dEyes[i] = (dLeft+dRight)/(2*dLeftRight);
        }
    }

    return dEyes;
}


/**
* @param    dEyess      - input     a vector of dEyes, refer to Cristinacce paper
* @param    wrongDet    - output    how many wrong detections
* @param    mindEyes    - input     min dEyes
* @param    maxdEyes    - input     max dEyes
* @param    nb          - input     how many evaluation levels to be used
*/
vector<int> CEvaluationAlgs::DEyesEval( const vector<double>& dEyess,
                                        unsigned int& wrongDet,
                                        double mindEyes,
                                        double maxdEyes,
                                        unsigned int nb )
{
    vector<int> res(nb, 0);
    double interval = (maxdEyes-mindEyes)/(double)nb;
    double curdEye = 0.0;

    for(unsigned int i = 0; i < nb; i++)
    {
        curdEye = mindEyes + i*interval;
        for (unsigned int j = 0; j < dEyess.size(); j++)
        {
            if (dEyess[j] < curdEye)
            {
                res[i]++;
            }
        }
    }

    wrongDet = 0;
    for (unsigned int j = 0; j < dEyess.size(); j++)
    {
        if (dEyess[j] > maxdEyes)
        {
            wrongDet++;
        }
    }

    return res;
}


/**
* @param    detectedFaceComp    - input     detected face components
* @param    faceCompCenters     - input     loaded face components' centers (realistic face information)
* @return   a vector of MSEs
*/
vector<double> CEvaluationAlgs::MSEFaceComp(
    const vector<Rect>& detectedFaceComp,
    const vector<Point2d>& faceCompCenters )
{
    assert(detectedFaceComp.size() == faceCompCenters.size() );
    unsigned int size = detectedFaceComp.size();
    double xCenter, yCenter;
    vector<double> mse(size, 0.0);

    for(unsigned int i = 0; i < size; i++)
    {
        xCenter = detectedFaceComp[i].x + (double)detectedFaceComp[i].width/2.0;
        yCenter = detectedFaceComp[i].y + (double)detectedFaceComp[i].height/2.0;
        mse[i]  = sqrt ( pow( (xCenter-faceCompCenters[i].x), 2.0)
            + pow( (yCenter-faceCompCenters[i].y), 2.0) );
    }

    return mse;
}

