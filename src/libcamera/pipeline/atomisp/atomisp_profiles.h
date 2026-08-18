/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

#include <string>

#include "atomisp_helpers.h"

namespace libcamera {

struct AtomispAeTuning {
	unsigned int cadenceInterval;
	double targetMsv;
	double satisfactoryMsv;
	double proportionalGain;
	double maximumStep;
	double lowLightProportionalGain;
	double lowLightMaximumStep;
	double lowLightMsv;
	int lowLightGainFloor;
	int frameLengthLinesMaximum;
	int maximumVblankFactor;
	int initialGain;
	bool processFirstFrame;
};

struct AtomispCameraProfile {
	bool softwareAe;
	bool sensorFrameLength;
	unsigned int minimumCaptureWidth;
	unsigned int captureSizeDelta;
	AtomispAeTuning aeTuning;
};

inline AtomispCameraProfile atomispCameraProfile(unsigned int hwRevision,
						 const std::string &sensorModel)
{
	return {
		atomispSupportsSoftwareAe(hwRevision),
		sensorModel == "mt9m114",
		1000,
		16,
		{
			15,
			2.5,
			0.3,
			0.02,
			0.10,
			0.08,
			0.25,
			1.2,
			128,
			65535,
			45,
			sensorModel == "mt9m114" ? 511 : 0,
			sensorModel == "mt9m114",
		},
	};
}

} /* namespace libcamera */
