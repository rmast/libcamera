/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

#include <string>

#include "atomisp_helpers.h"

namespace libcamera {

struct AtomispCameraProfile {
	bool softwareAe;
	bool sensorFrameLength;
	unsigned int minimumCaptureWidth;
	unsigned int captureSizeDelta;
};

inline AtomispCameraProfile atomispCameraProfile(unsigned int hwRevision,
						 const std::string &sensorModel)
{
	return {
		atomispSupportsSoftwareAe(hwRevision),
		sensorModel == "mt9m114",
		1000,
		16,
	};
}

} /* namespace libcamera */
