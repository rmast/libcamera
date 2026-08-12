/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

namespace libcamera {

constexpr bool atomispAeCadenceFrame(unsigned int frame, unsigned int interval)
{
	return (frame + 1) % interval == 0;
}

} /* namespace libcamera */
