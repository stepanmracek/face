/*
 * realsenseprocessor.h
 *
 *  Created on: 1. 5. 2015
 *      Author: dron
 */

#ifndef FACELIB_REALSENSEPROCESSOR_H_
#define FACELIB_REALSENSEPROCESSOR_H_

#include <pxcprojection.h>
#include "faceCommon/linalg/common.h"

namespace Face {
namespace Sensors {
namespace RealSense {

	class RealSenseProcessor {
		public:
			/// return image masked by depth map
			static ImageGrayscale depthMask(PXCImage *depth, PXCImage *color, PXCProjection* projection, const ImageGrayscale& img);

			/// fill holes in specified roi of mask with pixel values from img 
			/// - add some margin pixels (margin expected in <0.0, 1.0> range)
			static ImageGrayscale fillHoles(const cv::Rect& roi, const ImageGrayscale& mask, const ImageGrayscale& img, double margin);
	};

}
}
}



#endif /* FACELIB_REALSENSEPROCESSOR_H_ */
