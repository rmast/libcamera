/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

#include <algorithm>

#include <libcamera/base/log.h>
#include <libcamera/geometry.h>

namespace libcamera {

constexpr unsigned int kAtomispHwRevisionMask = 0x0000ff00;
constexpr unsigned int kAtomispHwRevisionIsp2401 = 0x00002000;

inline Size atomispAdjustSize(const Size &requestedSize,
			      const SizeRange &supportedSizes)
{
	ASSERT(supportedSizes.min <= supportedSizes.max);

	if (supportedSizes.min == supportedSizes.max)
		return supportedSizes.max;

	unsigned int hStep = supportedSizes.hStep;
	unsigned int vStep = supportedSizes.vStep;

	if (hStep == 0)
		hStep = supportedSizes.max.width - supportedSizes.min.width;
	if (vStep == 0)
		vStep = supportedSizes.max.height - supportedSizes.min.height;

	Size adjusted = requestedSize.boundedTo(supportedSizes.max)
				.expandedTo(supportedSizes.min);

	return adjusted.shrunkBy(supportedSizes.min)
		.alignedDownTo(hStep, vStep)
		.grownBy(supportedSizes.min);
}

constexpr unsigned int atomispAlignedStride(unsigned int stride)
{
	return ((stride + 63) / 64) * 64;
}

constexpr bool atomispAeCadenceFrame(unsigned int frame, unsigned int interval)
{
	return (frame + 1) % interval == 0;
}

constexpr bool atomispAeProcessFrame(unsigned int frame, unsigned int interval,
					     bool processFirstFrame)
{
	return (processFirstFrame && frame == 0) ||
	       atomispAeCadenceFrame(frame, interval);
}

constexpr unsigned int atomispAeCadenceInterval(unsigned int interval,
					 unsigned int normalFrameLength,
					 unsigned int frameLength)
{
	if (!normalFrameLength || !frameLength)
		return interval;

	return std::max(1U, interval * normalFrameLength / frameLength);
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

constexpr int atomispTargetVblank(int height, int vblank,
				  double measuredMsv, double targetMsv,
				  int maximum)
{
	if (measuredMsv <= 0.0)
		return maximum;

	int frameLength = height + vblank;
	int targetFrameLength = static_cast<int>(frameLength * targetMsv / measuredMsv);

	return std::clamp(std::max(targetFrameLength - height, vblank + 1),
			  vblank, maximum);
}

constexpr int atomispTargetVblankDown(int height, int vblank,
				      double measuredMsv, double targetMsv,
				      int minimum)
{
	int frameLength = height + vblank;
	int targetFrameLength = static_cast<int>(frameLength * targetMsv / measuredMsv);

	return std::clamp(std::min(targetFrameLength - height, vblank - 1),
			  minimum, vblank);
}

constexpr int atomispNextExposure(int exposure, int minimum, int maximum,
				  double measuredMsv, double lowLightMsv,
				  double factor)
{
	if (measuredMsv < lowLightMsv)
		return maximum;

	int nextExposure = static_cast<int>(exposure * factor);

	return std::clamp(std::max(nextExposure, exposure + 1), minimum, maximum);
}

constexpr int atomispTargetGain(int gain, double measuredMsv,
				double targetMsv, int maximum)
{
	if (measuredMsv <= 0.0)
		return maximum;

	int targetGain = static_cast<int>(gain * targetMsv / measuredMsv);

	return std::clamp(std::max(targetGain, gain + 1), gain, maximum);
}

} /* namespace libcamera */
