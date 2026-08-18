/* SPDX-License-Identifier: LGPL-2.1-or-later */

#pragma once

#include <optional>
#include <string_view>

inline bool v4l2CompatNeedsPackedPaddingWidthQuirk(const std::optional<std::string_view> &model)
{
	return model && *model == "mt9m114";
}

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
