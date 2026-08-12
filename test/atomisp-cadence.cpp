/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "../src/libcamera/pipeline/atomisp/atomisp_helpers.h"

#include <iostream>

#include "test.h"

using namespace libcamera;

class AtomispCadenceTest : public Test
{
protected:
	int run()
	{
		for (unsigned int frame = 0; frame <= 30; ++frame) {
			bool expected = frame == 0 || frame == 15 || frame == 30;
			if (atomispAeCadenceFrame(frame, 15) != expected) {
				std::cerr << "Unexpected cadence decision for frame "
					  << frame << std::endl;
				return TestFail;
			}
		}

		return TestPass;
	}
};

TEST_REGISTER(AtomispCadenceTest)
