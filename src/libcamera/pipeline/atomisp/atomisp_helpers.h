/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

#include <algorithm>

namespace libcamera {

constexpr unsigned int kAtomispHwRevisionMask = 0x0000ff00;
constexpr unsigned int kAtomispHwRevisionIsp2401 = 0x00002000;

constexpr bool atomispAeCadenceFrame(unsigned int frame, unsigned int interval)
{
	return (frame + 1) % interval == 0;
}

constexpr bool atomispSupportsSoftwareAe(unsigned int hwRevision)
{
	return (hwRevision & kAtomispHwRevisionMask) == kAtomispHwRevisionIsp2401;
}

constexpr int atomispNextVblank(int height, int vblank, double factor,
				int maximum)
{
	int frameLength = height + vblank;
	int nextFrameLength = static_cast<int>(frameLength * factor);

	return std::clamp(std::max(nextFrameLength - height, vblank + 1),
			  vblank, maximum);
}

constexpr int atomispNextGain(int gain, double factor, int minimum,
			     int maximum)
{
	int nextGain = static_cast<int>(gain * factor);

	return std::clamp(std::max(nextGain, gain + 1), minimum, maximum);
}

} /* namespace libcamera */
