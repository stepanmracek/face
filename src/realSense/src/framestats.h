/*
 * FrameStats.h
 *
 *  Created on: 9. 5. 2015
 *      Author: dron
 */

#ifndef FACELIB_FRAMESTATS_H_
#define FACELIB_FRAMESTATS_H_

#include <Poco/Timestamp.h>

namespace Face {

	class FrameStats {
		public:
			FrameStats(int averageCount);
			virtual ~FrameStats();
			void reset();

			void update();

			int frameMs() const;
			int fps() const;

		private:
			int frameMs_;
			Poco::Timestamp timer;
			int frameCounter;
			int averageCount;

	};

} /* namespace Face */

#endif /* FACELIB_FRAMESTATS_H_ */
