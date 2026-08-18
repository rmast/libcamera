/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "../src/libcamera/pipeline/atomisp/atomisp_helpers.h"
#include "../src/libcamera/pipeline/atomisp/atomisp_profiles.h"

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

		if (!atomispAeProcessFrame(0, 15, true) ||
		    !atomispAeProcessFrame(14, 15, true) ||
		    atomispAeProcessFrame(1, 15, true) ||
		    atomispAeProcessFrame(0, 15, false)) {
			std::cerr << "Unexpected initial AtomISP AE cadence" << std::endl;
			return TestFail;
		}

		if (atomispAeCadenceInterval(15, 997, 997) != 15 ||
		    atomispAeCadenceInterval(15, 997, 8000) != 1) {
			std::cerr << "Unexpected AtomISP cadence interval" << std::endl;
			return TestFail;
		}

		if (atomispSupportsSoftwareAe(0x1010) ||
		    !atomispSupportsSoftwareAe(0x2000) ||
		    !atomispSupportsSoftwareAe(0x2010)) {
			std::cerr << "Unexpected AtomISP AE capability" << std::endl;
			return TestFail;
		}

		const AtomispCameraProfile mt9m114 =
			atomispCameraProfile(0x2000, "mt9m114");
		const AtomispCameraProfile otherSensor =
			atomispCameraProfile(0x1010, "ov2685");
		if (!mt9m114.softwareAe || !mt9m114.sensorFrameLength ||
		    otherSensor.softwareAe || otherSensor.sensorFrameLength ||
		    mt9m114.minimumCaptureWidth != 1000 ||
		    mt9m114.captureSizeDelta != 16 ||
		    mt9m114.aeTuning.cadenceInterval != 15 ||
		    mt9m114.aeTuning.targetMsv != 2.5 ||
		    mt9m114.aeTuning.satisfactoryMsv != 0.3 ||
		    mt9m114.aeTuning.proportionalGain != 0.02 ||
		    mt9m114.aeTuning.maximumStep != 0.10 ||
		    mt9m114.aeTuning.lowLightProportionalGain != 0.08 ||
		    mt9m114.aeTuning.lowLightMaximumStep != 0.25 ||
		    mt9m114.aeTuning.lowLightMsv != 1.2 ||
		    mt9m114.aeTuning.lowLightGainFloor != 128 ||
		    mt9m114.aeTuning.frameLengthLinesMaximum != 65535 ||
		    mt9m114.aeTuning.maximumVblankFactor != 45 ||
		    mt9m114.aeTuning.initialGain != 511 ||
		    !mt9m114.aeTuning.processFirstFrame ||
		    otherSensor.aeTuning.initialGain != 0 ||
		    otherSensor.aeTuning.processFirstFrame) {
			std::cerr << "Unexpected AtomISP camera profile" << std::endl;
			return TestFail;
		}

		if (atomispNextVblank(976, 21, 1.2, 43889) != 220 ||
		    atomispNextVblank(976, 43889, 1.2, 43889) != 43889) {
			std::cerr << "Unexpected AtomISP frame-length step" << std::endl;
			return TestFail;
		}

		if (atomispTargetVblank(976, 21, 0.15, 1.2, 43889) != 6999 ||
		    atomispTargetVblank(976, 21, 0.5, 2.5, 43889) != 4009 ||
		    atomispTargetVblank(976, 21, 0.0, 1.2, 43889) != 43889) {
			std::cerr << "Unexpected AtomISP target frame length" << std::endl;
			return TestFail;
		}

		if (atomispTargetVblankDown(976, 43889, 5.0, 2.5, 21) != 21456 ||
		    atomispTargetVblankDown(976, 50, 2.5, 2.5, 21) != 49) {
			std::cerr << "Unexpected AtomISP bright frame length" << std::endl;
			return TestFail;
		}

		if (atomispNextExposure(497, 1, 995, 0.02, 1.2, 1.05) != 995 ||
		    atomispNextExposure(497, 1, 995, 1.5, 1.2, 1.05) != 521) {
			std::cerr << "Unexpected AtomISP exposure step" << std::endl;
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
