/*
 * FrameStats.cpp
 *
 *  Created on: 9. 5. 2015
 *      Author: dron
 */

#include "framestats.h"

namespace Face {

	FrameStats::FrameStats(int averageCount) : averageCount(averageCount) {
		reset();
	}

	FrameStats::~FrameStats() {
	}

	void FrameStats::reset() {
		timer.update();
		frameCounter = 0;
		frameMs_ = 1000;
	}

	void FrameStats::update() {
		if (++frameCounter % averageCount == 0) {
			frameMs_ = timer.elapsed() / 1000 / frameCounter;
			timer.update();
			frameCounter = 0;
		}
	}

	int FrameStats::frameMs() const {
		return frameMs_;
	}

	int FrameStats::fps() const {
		return frameMs_ ? 1000 / frameMs_ : 0;
	}

} /* namespace Face */
