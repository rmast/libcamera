/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "../src/v4l2/v4l2_camera_proxy_helpers.h"

#include <iostream>

#include "test.h"

class V4L2CameraProxyTest : public Test
{
protected:
	int run()
	{
		if (v4l2CompatExposedWidth(true, false, 1296, 2624) != 1296 ||
		    v4l2CompatExposedWidth(true, true, 1296, 2624) != 1312 ||
		    v4l2CompatExposedWidth(true, true, 1296, 2592) != 1296 ||
		    v4l2CompatExposedWidth(false, true, 1296, 2624) != 1296) {
			std::cerr << "Unexpected V4L2 compatibility width" << std::endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(V4L2CameraProxyTest)
