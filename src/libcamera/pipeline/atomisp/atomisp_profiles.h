/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

#include <string>

#include "atomisp_helpers.h"

namespace libcamera {

struct AtomispCameraProfile {
	bool softwareAe;
	bool sensorFrameLength;
};

inline AtomispCameraProfile atomispCameraProfile(unsigned int hwRevision,
						 const std::string &sensorModel)
{
	return {
		atomispSupportsSoftwareAe(hwRevision),
		sensorModel == "mt9m114",
	};
}

} /* namespace libcamera */
