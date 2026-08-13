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
			bool expected = frame == 14 || frame == 29;
			if (atomispAeCadenceFrame(frame, 15) != expected) {
				std::cerr << "Unexpected cadence decision for frame "
					  << frame << std::endl;
				return TestFail;
			}
		}

		if (atomispSupportsSoftwareAe(0x1010) ||
		    !atomispSupportsSoftwareAe(0x2000) ||
		    !atomispSupportsSoftwareAe(0x2010)) {
			std::cerr << "Unexpected AtomISP AE capability" << std::endl;
			return TestFail;
		}

		if (atomispNextVblank(976, 21, 1.2, 43889) != 220 ||
		    atomispNextVblank(976, 43889, 1.2, 43889) != 43889) {
			std::cerr << "Unexpected AtomISP frame-length step" << std::endl;
			return TestFail;
		}

		if (atomispTargetGain(32, 0.45, 2.5, 511) != 177 ||
		    atomispTargetGain(177, 1.65, 2.5, 511) != 268 ||
		    atomispTargetGain(177, 0.0, 2.5, 511) != 511) {
			std::cerr << "Unexpected AtomISP target gain" << std::endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(AtomispCadenceTest)
