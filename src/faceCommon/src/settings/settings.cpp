/*
 * settings.cpp
 *
 *  Created on: 29. 4. 2015
 *      Author: dron
 */
#include "faceCommon/settings/settings.h"

#include <iostream>

namespace Face {
	const std::string Settings::DebugKey = "DebugKey";
	const std::string Settings::CascadeFaceDetectorPathKey = "CascadeFaceDetectorPathKey";
	const std::string Settings::ShapePredictorPathKey = "ShapePredictorPathKey";
    const std::string Settings::MeanFaceModelPathKey = "MeanFaceModelPathKey";
    const std::string Settings::MeanFaceModelLandmarksPathKey = "MeanFaceModelLandmarksPath";
    const std::string Settings::PreAlignTemplatePathKey = "PreAlignTemplatePath";
    const std::string Settings::MultiExtractorPathKey = "MultiExtractorPath";


	Poco::Mutex Settings::m;

	Settings& Settings::instance() {
		Poco::Mutex::ScopedLock l(m);
		static Settings settings;
		return settings;
	}

	bool Settings::debug() const {
		auto found = settingsMap.find(DebugKey);
		if (found != settingsMap.end()) {
			return found->second.convert<bool>();
		};

		return false;
	}

    Settings::Settings()
    {
		settingsMap[CascadeFaceDetectorPathKey] = "haar-face.xml";
		settingsMap[ShapePredictorPathKey] = "shape_predictor_68_face_landmarks.dat";
        settingsMap[MeanFaceModelPathKey] = "meanForAlign.obj";
        settingsMap[MeanFaceModelLandmarksPathKey] = "meanForAlign.yml";
        settingsMap[PreAlignTemplatePathKey] = "preAlignTemplate.yml";
        settingsMap[MultiExtractorPathKey] = "softKinetic/scaled-trained";
    }
}

