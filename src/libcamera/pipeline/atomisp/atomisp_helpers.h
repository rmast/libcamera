/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

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

} /* namespace libcamera */
