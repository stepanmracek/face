/*
 * Settings.h
 *
 *  Created on: 29. 4. 2015
 *      Author: dron
 */

#ifndef FACECOMMON_SETTINGS_H_
#define FACECOMMON_SETTINGS_H_

#include <map>
#include <string>
#include <Poco/DynamicAny.h>
#include <Poco/Mutex.h>
#include "faceCommon/faceCommon.h"

namespace Face {

	class FACECOMMON_EXPORTS Settings {
		public:
			static const std::string DebugKey;
            static const std::string CascadeFaceDetectorPathKey;
			static const std::string ShapePredictorPathKey;
            static const std::string MeanFaceModelPathKey;
            static const std::string MeanFaceModelLandmarksPathKey;
            static const std::string PreAlignTemplatePathKey;
            static const std::string MultiExtractorPathKey;

			static Settings& instance();

			std::map<std::string, Poco::DynamicAny> settingsMap;

			bool debug() const;

		private:
            Settings();
			static Poco::Mutex m;
	};
}



#endif /* FACECOMMON_SETTINGS_H_ */
