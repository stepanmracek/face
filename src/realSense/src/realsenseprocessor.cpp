/*
 * realsenseprocessor.cpp
 *
 *  Created on: 1. 5. 2015
 *      Author: dron
 */

#include "realsenseprocessor.h"


namespace Face {
namespace Sensors {
namespace RealSense {

	ImageGrayscale RealSenseProcessor::depthMask(PXCImage *depth, PXCImage *color, PXCProjection* projection, const ImageGrayscale& img) {
		ImageGrayscale maskedGrayPreview(img.clone());

		PXCImage* rsColorMask = projection->CreateDepthImageMappedToColor(depth, color);
		if (rsColorMask) {
			PXCImage::ImageInfo info = rsColorMask->QueryInfo();

			PXCImage::ImageData colorData;
			pxcStatus s = rsColorMask->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &colorData);
			if (s == PXC_STATUS_NO_ERROR) {
				unsigned char* colorDataPtr = reinterpret_cast<unsigned char*>(colorData.planes[0]);

				cv::Mat color(480, 640, CV_8UC4, colorDataPtr);
				rsColorMask->ReleaseAccess(&colorData);
				cv::cvtColor(color, maskedGrayPreview, CV_RGB2GRAY);
				img.copyTo(maskedGrayPreview, maskedGrayPreview);
			}

			rsColorMask->Release();
		}

		//cv::imshow("grayscalePreview mask", maskedGrayPreview);
		return maskedGrayPreview;
	}

	ImageGrayscale RealSenseProcessor::fillHoles(const cv::Rect& roi, const ImageGrayscale& mask, const ImageGrayscale& img, double margin) {
		ImageGrayscale processed(mask.clone());

		auto fill = [=](int from1, int to1, int from2, int to2, bool rowsFirst, ImageGrayscale& processed) -> void {
			double _margin = std::min(std::max(margin, 0.0), 1.0);
			for (int i1 = from1; i1 < to1; ++i1) {
				int begin = from2;
				int end = to2;
				for (int i2 = begin; i2 < end; ++i2) {
					int row = rowsFirst ? i1 : i2;
					int col = rowsFirst ? i2 : i1;
					if (mask(row, col) > 0) {
						begin = i2;
						break;
					}
				}
				for (int i2 = end; i2 > begin; --i2) {
					int row = rowsFirst ? i1 : i2;
					int col = rowsFirst ? i2 : i1;
					if (mask(row, col) > 0) {
						end = i2;
						break;
					}
				}

				if (end > begin) {
					int offset = static_cast<int>((end - begin) * _margin * 0.5);
					begin = std::max(0, begin - offset);
					end = std::min(
						rowsFirst ? processed.rows : processed.cols,
						end + offset);

					for (int i2 = begin; i2 < end; ++i2) {
						int row = rowsFirst ? i1 : i2;
						int col = rowsFirst ? i2 : i1;
						processed(row, col) = img(row, col);
					}
				}
			}
		};

		fill(roi.y, roi.y + roi.height, roi.x, roi.x + roi.width, true, processed);
		//fill(roi.x, roi.x + roi.width, roi.y, roi.y + roi.height, false, processed);


		return processed;
	}
}
}
}
