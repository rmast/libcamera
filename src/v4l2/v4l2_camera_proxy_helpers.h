/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

constexpr unsigned int v4l2CompatExposedWidth(bool packedYuv,
					      unsigned int activeWidth,
					      unsigned int stride)
{
	if (packedYuv && stride > activeWidth * 2)
		return stride / 2;

	return activeWidth;
}
