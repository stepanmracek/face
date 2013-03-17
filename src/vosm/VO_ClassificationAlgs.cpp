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


#include <iostream>
#include <cstdio>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "VO_ClassificationAlgs.h"


/************************************************************************/
/*@author       JIA Pei                                                 */
/*@version      2010-11-04                                              */
/*@brief        Training                                                */
/*@param        data        Input - input data                          */
/*@param        categories  Input - a column vector                     */
/*@return       void                                                    */
/************************************************************************/
void CClassificationAlgs::Training( const Mat_<float>& data,
                                    const Mat_<int>& categories)
{
    unsigned int NbOfSamples = data.rows;
    set<int> ClassSet;
    for(int i = 0; i < categories.rows; i++)
    {
        ClassSet.insert(categories(i, 0));
    }
    this->m_iNbOfCategories = ClassSet.size();

    switch(this->m_iClassificationMethod)
    {
    case CClassificationAlgs::DecisionTree:
        this->m_CVDtree.train(  data,
                                CV_ROW_SAMPLE,
                                categories,
                                Mat(),
                                Mat(),
                                Mat(),
                                Mat(),
                                CvDTreeParams(  INT_MAX,
                                                2,
                                                0.0f,
                                                false,
                                                this->m_iNbOfCategories,
                                                0,
                                                false,
                                                false,
                                                0 ) );
        break;
    case CClassificationAlgs::Boost:
        this->m_CVBoost.train(  data,
                                CV_ROW_SAMPLE,
                                categories,
                                Mat(),
                                Mat(),
                                Mat(),
                                Mat(),
                                CvBoostParams(  CvBoost::DISCRETE,
                                                50,
                                                0.95,
                                                INT_MAX,
                                                false,
                                                0),
                                false );
        break;
    case CClassificationAlgs::RandomForest:
        this->m_CVRTrees.train( data, 
                                CV_ROW_SAMPLE,
                                categories,
                                Mat(),
                                Mat(),
                                Mat(),
                                Mat(),
                                CvRTParams( INT_MAX,
                                            2,
                                            0.0f,
                                            false,
                                            this->m_iNbOfCategories,
                                            0,
                                            true,
                                            0,
                                            100,
                                            0.0f,
                                            CV_TERMCRIT_ITER ) );
        break;
    case CClassificationAlgs::ExtremeRandomForest:
        this->m_CVERTrees.train(data,
                                CV_ROW_SAMPLE,
                                categories,
                                Mat(),
                                Mat(),
                                Mat(),
                                Mat(),
                                CvRTParams( INT_MAX,
                                            2,
                                            0.0f,
                                            false,
                                            this->m_iNbOfCategories,
                                            0,
                                            true,
                                            0,
                                            100,
                                            0.0f,
                                            CV_TERMCRIT_ITER ) );
        break;
    case CClassificationAlgs::SVM:
        this->m_CVSVM.train(data,
                            categories,
                            Mat(),
                            Mat(),
                            CvSVMParams(CvSVM::C_SVC,
                                        CvSVM::RBF,
                                        0,
                                        1,
                                        0,
                                        1,
                                        0,
                                        0,
                                        NULL,
                                        cvTermCriteria( CV_TERMCRIT_ITER,
                                                        1000,
                                                        1E-6) ) );
        break;
    }
}


/************************************************************************/
/*@author       JIA Pei                                                 */
/*@version      2010-11-04                                              */
/*@brief        Classification                                          */
/*@param        sample      Input - a sample to be classified           */
/*@return       the classified category                                 */
/************************************************************************/
int CClassificationAlgs::Classification(const Mat_<float>& sample )
{
    int res = -1;
    switch(this->m_iClassificationMethod)
    {
    case CClassificationAlgs::DecisionTree:
        {
            CvDTreeNode* node = this->m_CVDtree.predict( sample );
            res = node->class_idx;
        }
        break;
    case CClassificationAlgs::Boost:
        {
            res = (int) this->m_CVBoost.predict( sample );
        }
        break;
    case CClassificationAlgs::RandomForest:
        {
            res = (int) this->m_CVRTrees.predict( sample );
        }
        break;
    case CClassificationAlgs::ExtremeRandomForest:
        {
            res = (int) this->m_CVERTrees.predict( sample );
        }
        break;
    case CClassificationAlgs::SVM:
    default:
        {
            res = (int) this->m_CVSVM.predict( sample );
        }
        break;
    }

    return res;
}


/** Save the classifier */
void CClassificationAlgs::Save(const string& fn ) const
{
    switch(this->m_iClassificationMethod)
    {
    case CClassificationAlgs::DecisionTree:
        {
            this->m_CVDtree.save(fn.c_str());
        }
        break;
    case CClassificationAlgs::Boost:
        {
            this->m_CVBoost.save(fn.c_str());
        }
        break;
    case CClassificationAlgs::RandomForest:
        {
            this->m_CVRTrees.save(fn.c_str());
        }
        break;
    case CClassificationAlgs::ExtremeRandomForest:
        {
            this->m_CVERTrees.save(fn.c_str());
        }
        break;
    case CClassificationAlgs::SVM:
    default:
        {
            this->m_CVSVM.save(fn.c_str());
        }
        break;
    }
}


/** Load the classifier */
void CClassificationAlgs::Load(const string& fn)
{
    switch(this->m_iClassificationMethod)
    {
    case CClassificationAlgs::DecisionTree:
        {
            this->m_CVDtree.load(fn.c_str());
        }
        break;
    case CClassificationAlgs::Boost:
        {
            this->m_CVBoost.load(fn.c_str());
        }
        break;
    case CClassificationAlgs::RandomForest:
        {
            this->m_CVRTrees.load(fn.c_str());
        }
        break;
    case CClassificationAlgs::ExtremeRandomForest:
        {
            this->m_CVERTrees.load(fn.c_str());
        }
        break;
    case CClassificationAlgs::SVM:
        {
            this->m_CVSVM.load(fn.c_str());
        }
    case CClassificationAlgs::NONE:
    default:
        break;
    }
}

