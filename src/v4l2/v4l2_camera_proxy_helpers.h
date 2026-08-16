/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

constexpr unsigned int v4l2CompatExposedWidth(bool packedYuv,
					      bool applyPackedPaddingWidthQuirk,
					      unsigned int activeWidth,
					      unsigned int stride)
{
	if (applyPackedPaddingWidthQuirk && packedYuv &&
	    stride > activeWidth * 2)
		return stride / 2;

	return activeWidth;
}
