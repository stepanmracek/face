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

#ifndef __VO_ASMNDPROFILES_H__
#define __VO_ASMNDPROFILES_H__


#include <vector>

#include "opencv/cv.h"
#include "opencv/highgui.h"

#include "VO_Profile.h"
#include "VO_AXM.h"


using namespace std;
using namespace cv;

/**
* @author   JIA Pei
* @brief    ND Profile ASM.
* @note     Current VO_ASMNDProfiles only deal with 1 channel gray image
*/
class VO_ASMNDProfiles : public VO_AXM
{
friend class VO_Fitting2DSM;
friend class VO_FittingAAMBasic;
friend class VO_FittingAAMForwardIA;
friend class VO_FittingAAMInverseIA;
friend class VO_FittingASMLTCs;
friend class VO_FittingASMNDProfiles;
friend class VO_FittingAFM;
protected:
    /** Number of profile pixels refer to Cootes "Statistical Models of Appearance for Computer Vision" page 38 */
    /** rgb should have 3 times profiles than gray */
    vector<unsigned int>                        m_iNbOfProfilesPerPixelAtLevels;

    /** Number of dimensions of profiles */
    unsigned int                                m_iNbOfProfileDim;

    /** All normalized profiles m_iNbOfSamples* m_iNbOfPyramidLevels*m_iNbOfPoints */
    vector< vector< vector< VO_Profile > > >    m_vvvNormalizedProfiles;

    /** Average normalized profile m_iNbOfPyramidLevels*m_iNbOfPoints */
    vector< vector< VO_Profile > >              m_vvMeanNormalizedProfile;

    /** inverse of Sg, m_iNbOfPyramidLevels*m_iNbOfPoints*m_iNbOfProfileDim*(m_iNbOfProfilesPerPixelAtLevels*m_iNbOfProfilesPerPixelAtLevels) refer to Cootes "Statistics Models of Appearance for Computer Vision", page 38 (7.2) */
    vector< vector< vector< Mat_<float> > > >   m_vvvCVMInverseOfSg;

    /** Initialization */
    void init()
    {
        this->m_iMethod = VO_AXM::ASM_PROFILEND;
        this->m_iNbOfProfilesPerPixelAtLevels.clear();
        this->m_vvvNormalizedProfiles.clear();
        this->m_vvMeanNormalizedProfile.clear();
        this->m_vvvCVMInverseOfSg.clear();
    }

public:
    enum { SAME = 0, PYRAMID = 1, DESCENDING = 2};

    /** Default constructor to create a VO_ASMNDProfiles object */
    VO_ASMNDProfiles() {this->init();}

    /** Destructor */
    ~VO_ASMNDProfiles()
    {
        this->m_iNbOfProfilesPerPixelAtLevels.clear();
        this->m_vvvNormalizedProfiles.clear();
        this->m_vvMeanNormalizedProfile.clear();
        this->m_vvvCVMInverseOfSg.clear();
    }

    /** Build ASM ND Profile model */
    void            VO_BuildASMNDProfiles(  const vector<string>& allLandmarkFiles4Training,
                                            const vector<string>& allImgFiles4Training,
                                            const string& shapeinfoFileName,
                                            unsigned int database,
                                            unsigned int channels = 1,
                                            unsigned int levels = 4,
                                            unsigned int profdim = 2,
                                            unsigned int kk = 8,
                                            int trm = VO_Features::DIRECT,
                                            float TPShape = 0.95f,
                                            bool useKnownTriangles = false);

    /** profiles calculation, refer to Cootes "Statistical Models of Appearance for Computer Vision" page 38 */
    void            VO_LoadProfileTrainingData();

    /** profiles calculation, refer to Cootes "Statistical Models of Appearance for Computer Vision" page 38 */
    void            VO_CalcStatistics4AllProfiles();

    /** This function determines how many elements are used to compose a single profile vector */
    static void     VO_ProduceLevelProfileNumbers(vector<unsigned int>& nbOfProfilesPerPixelAtLevels, unsigned int iLevel, unsigned int NbOfLevel0 = 17, unsigned int ProduceMethod = SAME);

    /** Save ASM ND Profile model, to a specified folder */
    void            VO_Save(const string& fd);

    /** Load all parameters */
    void            VO_Load(const string& fd);

    /** Load parameters for fitting */
    void            VO_LoadParameters4Fitting(const string& fd);
};

#endif  // __VO_ASMNDPROFILES_H__

