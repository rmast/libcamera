/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Laurent Pinchart
 * Copyright (C) 2019, Martijn Braam
 *
 * Pipeline handler for Intel AtomISP (ISP2400/ISP2401) camera pipelines
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <list>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <stdint.h>
#include <string.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <linux/media-bus-format.h>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/color_space.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/pixel_format.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/converter.h"
#include "libcamera/internal/delayed_controls.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/global_configuration.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/software_isp/software_isp.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(AtomispPipeline)

/* -----------------------------------------------------------------------------
 *
 * Overview
 * --------
 *
 * The AtomispPipelineHandler relies on generic kernel APIs to control a camera
 * device, without any device-specific code and with limited device-specific
 * static data.
 *
 * To qualify for support by the simple pipeline handler, a device shall
 *
 * - be supported by V4L2 drivers, exposing the Media Controller API, the V4L2
 *   subdev APIs and the media bus format-based enumeration extension for the
 *   VIDIOC_ENUM_FMT ioctl ;
 * - not expose any device-specific API from drivers to userspace ;
 * - include one or more camera sensor media entities and one or more video
 *   capture devices ;
 * - have a capture pipeline with linear paths from the camera sensors to the
 *   video capture devices ; and
 * - have an optional memory-to-memory device to perform format conversion
 *   and/or scaling, exposed as a V4L2 M2M device.
 *
 * As devices that require a specific pipeline handler may still match the
 * above characteristics, the simple pipeline handler doesn't attempt to
 * automatically determine which devices it can support. It instead relies on
 * an explicit list of supported devices, provided in the supportedDevices
 * array.
 *
 * When matching a device, the pipeline handler enumerates all camera sensors
 * and attempts, for each of them, to find a path to a video capture video node.
 * It does so by using a breadth-first search to find the shortest path from the
 * sensor device to a valid capture device. This is guaranteed to produce a
 * valid path on devices with one only option and is a good heuristic on more
 * complex devices to skip paths that aren't suitable for the simple pipeline
 * handler. For instance, on the IPU-based i.MX6, the shortest path will skip
 * encoders and image converters, and it will end in a CSI capture device.
 * A more complex graph search algorithm could be implemented if a device that
 * would otherwise be compatible with the pipeline handler isn't correctly
 * handled by this heuristic.
 *
 * Once the camera data instances have been created, the match() function
 * creates a V4L2VideoDevice or V4L2Subdevice instance for each entity used by
 * any of the cameras and stores them in AtomispPipelineHandler::entities_,
 * accessible by the AtomispCameraData class through the
 * AtomispPipelineHandler::subdev() and AtomispPipelineHandler::video() functions.
 * This avoids duplication of subdev instances between different cameras when
 * the same entity is used in multiple paths.
 *
 * Finally, all camera data instances are initialized to gather information
 * about the possible pipeline configurations for the corresponding camera. If
 * valid pipeline configurations are found, a Camera is registered for the
 * AtomispCameraData instance.
 *
 * Pipeline Traversal
 * ------------------
 *
 * During the breadth-first search, the pipeline is traversed from entity to
 * entity, by following media graph links from source to sink, starting at the
 * camera sensor.
 *
 * When reaching an entity (on its sink side), if the entity is a V4L2 subdev
 * that supports the streams API, the subdev internal routes are followed to
 * find the connected source pads. Otherwise all of the entity's source pads
 * are considered to continue the graph traversal. The pipeline handler
 * currently considers the default internal routes only and doesn't attempt to
 * setup custom routes. This can be extended if needed.
 *
 * The shortest path between the camera sensor and a video node is stored in
 * AtomispCameraData::entities_ as a list of AtomispCameraData::Entity structures,
 * ordered along the data path from the camera sensor to the video node. The
 * Entity structure stores a pointer to the MediaEntity, as well as information
 * about how it is connected in that particular path for later usage when
 * configuring the pipeline.
 *
 * Pipeline Configuration
 * ----------------------
 *
 * The simple pipeline handler configures the pipeline by propagating V4L2
 * subdev formats from the camera sensor to the video node. The format is first
 * set on the camera sensor's output, picking a resolution supported by the
 * sensor that best matches the needs of the requested streams. Then, on every
 * link in the pipeline, the format is retrieved on the link source and set
 * unmodified on the link sink.
 *
 * The best sensor resolution is selected using a heuristic that tries to
 * minimize the required bus and memory bandwidth, as the simple pipeline
 * handler is typically used on smaller, less powerful systems. To avoid the
 * need to upscale, the pipeline handler picks the smallest sensor resolution
 * large enough to accommodate the need of all streams. Resolutions that
 * significantly restrict the field of view are ignored.
 *
 * When initializating the camera data, the above format propagation procedure
 * is repeated for every media bus format and size supported by the camera
 * sensor. Upon reaching the video node, the pixel formats compatible with the
 * media bus format are enumerated. Each combination of the input media bus
 * format, output pixel format and output size are recorded in an instance of
 * the AtomispCameraData::Configuration structure, stored in the
 * AtomispCameraData::configs_ vector.
 *
 * Format Conversion and Scaling
 * -----------------------------
 *
 * The capture pipeline isn't expected to include a scaler, and if a scaler is
 * available, it is ignored when configuring the pipeline. However, the simple
 * pipeline handler supports optional memory-to-memory converters to scale the
 * image and convert it to a different pixel format. If such a converter is
 * present, the pipeline handler enumerates, for each pipeline configuration,
 * the pixel formats and sizes that the converter can produce for the output of
 * the capture video node, and stores the information in the outputFormats and
 * outputSizes of the AtomispCameraData::Configuration structure.
 *
 * Concurrent Access to Cameras
 * ----------------------------
 *
 * The cameras created by the same pipeline handler instance may share hardware
 * resources. For instances, a platform may have multiple CSI-2 receivers but a
 * single DMA engine, prohibiting usage of multiple cameras concurrently. This
 * depends heavily on the hardware architecture, which the simple pipeline
 * handler has no a priori knowledge of. The pipeline handler thus implements a
 * heuristic to handle sharing of hardware resources in a generic fashion.
 *
 * Two cameras are considered to be mutually exclusive if they share common
 * pads along the pipeline from the camera sensor to the video node. An entity
 * can thus be used concurrently by multiple cameras, as long as pads are
 * distinct.
 *
 * A resource reservation mechanism is implemented by the AtomispPipelineHandler
 * acquirePipeline() and releasePipeline() functions to manage exclusive access
 * to pads. A camera reserves all the pads present in its pipeline when it is
 * started, and the start() function returns an error if any of the required
 * pads is already in use. When the camera is stopped, the pads it has reserved
 * are released.
 */

class AtomispPipelineHandler;

struct AtomispFrameInfo {
	AtomispFrameInfo(uint32_t f, Request *r, bool m)
		: frame(f), request(r), metadataRequired(m), metadataProcessed(false)
	{
	}

	uint32_t frame;
	Request *request;
	bool metadataRequired;
	bool metadataProcessed;
};

class AtomispFrames
{
public:
	void create(Request *request, bool metadataRequested);
	void destroy(uint32_t frame);
	void clear();

	AtomispFrameInfo *find(uint32_t frame);

private:
	std::map<uint32_t, AtomispFrameInfo> frameInfo_;
};

void AtomispFrames::create(Request *request, bool metadataRequired)
{
	const uint32_t frame = request->sequence();
	auto [it, inserted] = frameInfo_.try_emplace(frame, frame, request, metadataRequired);
	ASSERT(inserted);
}

void AtomispFrames::destroy(uint32_t frame)
{
	frameInfo_.erase(frame);
}

void AtomispFrames::clear()
{
	frameInfo_.clear();
}

AtomispFrameInfo *AtomispFrames::find(uint32_t frame)
{
	auto info = frameInfo_.find(frame);
	if (info == frameInfo_.end())
		return nullptr;
	return &info->second;
}

struct AtomispDriverInfo {
	const char *driver;
	/*
	 * Each converter in the list contains the name
	 * and the number of streams it supports.
	 */
	std::vector<std::pair<const char *, unsigned int>> converters;
	/*
	 * Using Software ISP is to be enabled per driver.
	 *
	 * The Software ISP can't be used together with the converters.
	 */
	bool swIspEnabled;
};

namespace {

static const AtomispDriverInfo supportedDevices[] = {
	{ "atomisp", {}, false },
	{ "atomisp-isp2", {}, false },
};

bool isRaw(const StreamConfiguration &cfg)
{
	return libcamera::PixelFormatInfo::info(cfg.pixelFormat).colourEncoding ==
	       libcamera::PixelFormatInfo::ColourEncodingRAW;
}

} /* namespace */

/*
 * Luminance-based AE loop for the AtomISP path. AtomISP outputs YUV, so
 * the debayer software-ISP path is not applicable. Instead, sample the Y
 * channel of each UYVY capture buffer and apply the same proportional
 * controller used by the soft-IPA AGC to adjust sensor exposure and gain.
 */
class AtomispAeLoop
{
public:
	Signal<const ControlList &> setSensorControls;

	bool configure(CameraSensor *sensor,
		       const PixelFormat &format, const Size &size,
		       unsigned int bpl);
	void bootstrap();
	void processBuffer(uint32_t sequence, FrameBuffer *buffer);

private:
	void updateExposure(double msv);

	CameraSensor *sensor_ = nullptr;
	PixelFormat format_;
	Size size_;

	int32_t exposure_ = 0, exposureMin_ = 0, exposureMax_ = 0;
	int32_t vblank_ = 0, vblankMin_ = 0, vblankMax_ = 0, vblankPracticalMax_ = 0;
	/* Sensor active frame height derived from vblank max (65535 - vblank_max). */
	int32_t height_ = 0;
	double gain_ = 1.0, gainMin_ = 1.0, gainMax_ = 1.0;
	unsigned int bpl_ = 0;

	/* Process every kInterval frames; slow enough for sensor response */
	static constexpr unsigned int kInterval = 15;
	/* MSV target and controller constants */
	static constexpr double kOptimalMsv = 2.5;
	static constexpr double kSatisfactory = 0.3;
	static constexpr double kPGain = 0.02;
	static constexpr double kMaxStep = 0.10;
	/* MT9M114 CAM_SENSOR_CFG_FRAME_LENGTH_LINES_MAX */
	static constexpr int32_t kFllMax = 65535;
	/* Limit vblank to this multiple of normal FLL to keep fps ≥ 1/kMaxVblankFactor. */
	static constexpr int32_t kMaxVblankFactor = 6;

	bool warnedUnsupportedFormat_ = false;
	unsigned int sampleCount_ = 0;
};

bool AtomispAeLoop::configure(CameraSensor *sensor,
			      const PixelFormat &format, const Size &size,
			      unsigned int bpl)
{
	sensor_ = sensor;
	format_ = format;
	size_ = size;
	bpl_ = bpl;

	const ControlInfoMap &ctrls = sensor->controls();

	auto itExp = ctrls.find(V4L2_CID_EXPOSURE);
	if (itExp == ctrls.end()) {
		LOG(AtomispPipeline, Warning) << "AtomISP AE: no exposure control";
		return false;
	}
	exposureMin_ = itExp->second.min().get<int32_t>();
	exposureMax_ = itExp->second.max().get<int32_t>();

	auto itGain = ctrls.find(V4L2_CID_ANALOGUE_GAIN);
	if (itGain == ctrls.end()) {
		LOG(AtomispPipeline, Warning) << "AtomISP AE: no gain control";
		return false;
	}
	gainMin_ = itGain->second.min().get<int32_t>();
	gainMax_ = itGain->second.max().get<int32_t>();

	/*
	 * Derive frame height from vblank_max = FLL_MAX - height (driver formula).
	 * This gives the correct per-mode exposure ceiling, including binned modes
	 * where the static driver max (995) is wrong (e.g. 643 for a 624-line mode).
	 */
	auto itVblank = ctrls.find(V4L2_CID_VBLANK);
	if (itVblank != ctrls.end()) {
		vblankMin_ = itVblank->second.min().get<int32_t>();
		vblankMax_ = itVblank->second.max().get<int32_t>();
		vblank_ = vblankMin_;
		height_ = kFllMax - vblankMax_;
		exposureMax_ = height_ + vblank_ - 2;
		/* kMaxVblankFactor × normal FLL → fps drops to 1/kMaxVblankFactor at most. */
		vblankPracticalMax_ = std::min((height_ + vblankMin_) * kMaxVblankFactor - height_,
					   vblankMax_);
	}

	/*
	 * Seed from actual hardware values (volatile controls) to avoid resetting
	 * a user-set gain back to the driver default on the first bootstrap emit.
	 */
	std::array<uint32_t, 2> hwIds = { V4L2_CID_EXPOSURE, V4L2_CID_ANALOGUE_GAIN };
	ControlList hwCtrls = sensor->getControls(hwIds);
	if (!hwCtrls.empty()) {
		exposure_ = std::clamp(hwCtrls.get(V4L2_CID_EXPOSURE).get<int32_t>(),
				   exposureMin_, exposureMax_);
		gain_ = std::clamp(static_cast<double>(
				   hwCtrls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>()),
				   gainMin_, gainMax_);
	} else {
		exposure_ = exposureMax_ / 2;
		gain_ = gainMin_;
	}

	LOG(AtomispPipeline, Debug)
		<< "AtomISP AE configured: exp [" << exposureMin_ << ".."
		<< exposureMax_ << "] hw=" << exposure_
		<< " gain [" << gainMin_ << ".." << gainMax_ << "] hw=" << gain_
		<< " height=" << height_ << " vblankPracticalMax=" << vblankPracticalMax_
		<< " format=" << format_ << " size=" << size_ << " bpl=" << bpl_;

	return true;
}

void AtomispAeLoop::bootstrap()
{
	/* Emit initial controls so hardware starts at the known-safe default. */
	ControlList sensorCtrls(sensor_->controls());
	sensorCtrls.set(V4L2_CID_EXPOSURE, exposure_);
	sensorCtrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	if (height_ > 0)
		sensorCtrls.set(V4L2_CID_VBLANK, vblank_);
	setSensorControls.emit(sensorCtrls);
	LOG(AtomispPipeline, Debug)
		<< "AtomISP AE bootstrap: exp=" << exposure_
		<< " gain=" << static_cast<int32_t>(gain_)
		<< " vblank=" << vblank_;
}

void AtomispAeLoop::processBuffer(uint32_t sequence, FrameBuffer *buffer)
{
	if (sequence % kInterval != 0)
		return;

	const bool isUyvy = format_ == formats::UYVY;
	const bool isYuyv = format_ == formats::YUYV;
	const bool isLumaPlane0 =
		format_ == formats::NV12 || format_ == formats::NV21 ||
		format_ == formats::YUV420 || format_ == formats::YVU420 ||
		format_ == formats::NV16 || format_ == formats::YUV422 ||
		format_ == formats::YUV444;

	if (!isUyvy && !isYuyv && !isLumaPlane0)
	{
		if (!warnedUnsupportedFormat_) {
			LOG(AtomispPipeline, Warning)
				<< "AtomISP AE: unsupported capture format " << format_
				<< ", sampling disabled";
			warnedUnsupportedFormat_ = true;
		}
		return;
	}

	MappedFrameBuffer in(buffer, MappedFrameBuffer::MapFlag::Read);
	if (!in.isValid()) {
		LOG(AtomispPipeline, Warning) << "AtomISP AE: mmap failed";
		return;
	}

	const uint8_t *data = in.planes()[0].begin();
	/* Use the actual hardware bpl (ISP may add 64-byte padding). */
	const unsigned int stride = bpl_;

	/*
	 * Sample Y over a regular grid: every 8th row, every 64th pixel.
	 * For packed 4:2:2 formats (UYVY/YUYV), read Y from alternating bytes.
	 * For planar/semi-planar YUV formats, plane 0 is a pure Y plane.
	 */
	uint64_t ySum = 0;
	unsigned int count = 0;
	for (unsigned int row = 0; row < size_.height; row += 8) {
		const uint8_t *line = data + row * stride;

		if (isUyvy || isYuyv) {
			const unsigned int yOffset = isUyvy ? 1 : 0;
			for (unsigned int col = yOffset; col + 1 < stride; col += 128) {
				ySum += line[col];
				count++;
			}
		} else {
			for (unsigned int col = 0; col < size_.width && col < stride; col += 64) {
				ySum += line[col];
				count++;
			}
		}
	}

	if (!count)
		return;

	/* Scale mean Y (0-255) to MSV range (0-5) matching the soft-IPA AGC */
	double msv = static_cast<double>(ySum) / count * 5.0 / 255.0;
	if ((sampleCount_++ % 20) == 0)
		LOG(AtomispPipeline, Debug)
			<< "AtomISP AE sample: seq=" << sequence
			<< " msv=" << msv << " exp=" << exposure_
			<< " gain=" << static_cast<int32_t>(gain_)
			<< " vblank=" << vblank_;
	updateExposure(msv);
}

void AtomispAeLoop::updateExposure(double msv)
{
	double error = kOptimalMsv - msv;

	if (std::abs(error) <= kSatisfactory)
		return;

	double step = std::clamp(error * kPGain, -kMaxStep, kMaxStep);
	double factor = 1.0 + step;
	bool changed = false;

	if (factor > 1.0) {
		/* Too dark: raise exposure → raise gain → extend vblank (last resort) */
		if (exposure_ < exposureMax_) {
			int32_t next = static_cast<int32_t>(exposure_ * factor);
			exposure_ = std::clamp(std::max(next, exposure_ + 1),
					       exposureMin_, exposureMax_);
			changed = true;
		} else if (gain_ < gainMax_) {
			gain_ = std::min(gain_ * factor, gainMax_);
			changed = true;
		} else if (height_ > 0 && vblank_ < vblankPracticalMax_) {
			/* Extend frame time, keeping fps ≥ 1/kMaxVblankFactor of normal. */
			int32_t next = static_cast<int32_t>(vblank_ * factor);
			vblank_ = std::min(std::max(next, vblank_ + 1), vblankPracticalMax_);
			exposureMax_ = height_ + vblank_ - 2;
			exposure_ = exposureMax_;
			changed = true;
		}
	} else {
		/* Too bright: restore fps first, then gain, then exposure */
		if (height_ > 0 && vblank_ > vblankMin_) {
			int32_t next = static_cast<int32_t>(vblank_ * factor);
			vblank_ = std::max(std::min(next, vblank_ - 1), vblankMin_);
			exposureMax_ = height_ + vblank_ - 2;
			exposure_ = std::min(exposure_, exposureMax_);
			changed = true;
		} else if (gain_ > gainMin_) {
			gain_ = std::max(gain_ * factor, gainMin_);
			changed = true;
		} else if (exposure_ > exposureMin_) {
			int32_t next = static_cast<int32_t>(exposure_ * factor);
			exposure_ = std::clamp(std::min(next, exposure_ - 1),
					       exposureMin_, exposureMax_);
			changed = true;
		}
	}

	if (!changed)
		return;

	ControlList sensorCtrls(sensor_->controls());
	sensorCtrls.set(V4L2_CID_EXPOSURE, exposure_);
	sensorCtrls.set(V4L2_CID_ANALOGUE_GAIN, static_cast<int32_t>(gain_));
	if (height_ > 0)
		sensorCtrls.set(V4L2_CID_VBLANK, vblank_);
	setSensorControls.emit(sensorCtrls);

	LOG(AtomispPipeline, Debug)
		<< "AtomISP AE update: msv=" << msv
		<< " exp=" << exposure_ << " gain=" << static_cast<int32_t>(gain_)
		<< " vblank=" << vblank_ << " expMax=" << exposureMax_;
}

class AtomispCameraData : public Camera::Private
{
public:
	AtomispCameraData(AtomispPipelineHandler *pipe,
			 unsigned int numStreams,
			 MediaEntity *sensor);

	bool isValid() const { return sensor_ != nullptr; }
	AtomispPipelineHandler *pipe();

	int init();
	int setupLinks();
	int setupFormats(V4L2SubdeviceFormat *format,
			 V4L2Subdevice::Whence whence,
			 Transform transform = Transform::Identity);
	void imageBufferReady(FrameBuffer *buffer);
	void clearIncompleteRequests();

	unsigned int streamIndex(const Stream *stream) const
	{
		return stream - &streams_.front();
	}

	struct Entity {
		/* The media entity, always valid. */
		MediaEntity *entity;
		/*
		 * Whether or not the entity is a subdev that supports the
		 * routing API.
		 */
		bool supportsRouting;
		/*
		 * The local sink pad connected to the upstream entity, null for
		 * the camera sensor at the beginning of the pipeline.
		 */
		const MediaPad *sink;
		/*
		 * The local source pad connected to the downstream entity, null
		 * for the video node at the end of the pipeline.
		 */
		const MediaPad *source;
		/*
		 * The link on the source pad, to the downstream entity, null
		 * for the video node at the end of the pipeline.
		 */
		MediaLink *sourceLink;
	};

	struct Configuration {
		uint32_t code;
		Size sensorSize;
		PixelFormat captureFormat;
		Size captureSize;
		std::vector<PixelFormat> outputFormats;
		SizeRange outputSizes;
	};

	std::vector<Stream> streams_;
	Stream *rawStream_;

	/*
	 * All entities in the pipeline, from the camera sensor to the video
	 * node.
	 */
	std::list<Entity> entities_;
	std::unique_ptr<CameraSensor> sensor_;
	V4L2VideoDevice *video_;
	V4L2Subdevice *frameStartEmitter_;

	std::vector<Configuration> configs_;
	std::map<PixelFormat, std::vector<const Configuration *>> formats_;

	std::unique_ptr<DelayedControls> delayedCtrls_;

	std::vector<std::unique_ptr<FrameBuffer>> conversionBuffers_;
	struct RequestOutputs {
		Request *request;
		std::map<const Stream *, FrameBuffer *> outputs;
	};
	std::queue<RequestOutputs> conversionQueue_;
	bool useConversion_;

	std::unique_ptr<Converter> converter_;
	std::unique_ptr<SoftwareIsp> swIsp_;
	std::unique_ptr<AtomispAeLoop> atomispAe_;
	AtomispFrames frameInfo_;

	void setSensorControls(const ControlList &sensorControls);

private:
	void tryPipeline(unsigned int code, const Size &size);
	static std::vector<const MediaPad *> routedSourcePads(MediaPad *sink);

	void tryCompleteRequest(Request *request);
	void conversionInputDone(FrameBuffer *buffer);
	void conversionOutputDone(FrameBuffer *buffer);

	void ispStatsReady(uint32_t frame, uint32_t bufferId);
	void metadataReady(uint32_t frame, const ControlList &metadata);
};

class AtomispCameraConfiguration : public CameraConfiguration
{
public:
	AtomispCameraConfiguration(Camera *camera, AtomispCameraData *data);

	Status validate() override;

	const AtomispCameraData::Configuration *pipeConfig() const
	{
		return pipeConfig_;
	}

	bool needConversion() const { return needConversion_; }
	const Transform &combinedTransform() const { return combinedTransform_; }

private:
	static constexpr unsigned int kNumBuffersDefault = 4;
	static constexpr unsigned int kNumBuffersMax = 32;

	/*
	 * The AtomispCameraData instance is guaranteed to be valid as long as
	 * the corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	std::shared_ptr<Camera> camera_;
	AtomispCameraData *data_;

	const AtomispCameraData::Configuration *pipeConfig_;
	bool needConversion_;
	Transform combinedTransform_;
};

class AtomispPipelineHandler : public PipelineHandler
{
public:
	AtomispPipelineHandler(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;

	bool match(DeviceEnumerator *enumerator) override;

	V4L2VideoDevice *video(const MediaEntity *entity);
	V4L2Subdevice *subdev(const MediaEntity *entity);
	std::shared_ptr<MediaDevice> converter() { return converter_; }
	bool swIspEnabled() const { return swIspEnabled_; }
	bool atomispQuirks() const { return atomispQuirks_; }

protected:
	int queueRequestDevice(Camera *camera, Request *request) override;

private:
	static constexpr unsigned int kMaxQueuedRequestsDevice = 4;
	static constexpr unsigned int kNumInternalBuffers = 4;

	struct EntityData {
		std::unique_ptr<V4L2VideoDevice> video;
		std::unique_ptr<V4L2Subdevice> subdev;
		std::map<const MediaPad *, AtomispCameraData *> owners;
	};

	AtomispCameraData *cameraData(Camera *camera)
	{
		return static_cast<AtomispCameraData *>(camera->_d());
	}

	bool matchDevice(std::shared_ptr<MediaDevice> media,
			 const AtomispDriverInfo &info,
			 DeviceEnumerator *enumerator);

	std::vector<MediaEntity *> locateSensors(MediaDevice *media);
	static int resetRoutingTable(V4L2Subdevice *subdev);

	const MediaPad *acquirePipeline(AtomispCameraData *data);
	void releasePipeline(AtomispCameraData *data);

	std::map<const MediaEntity *, EntityData> entities_;

	std::shared_ptr<MediaDevice> converter_;
	bool swIspEnabled_;
	bool atomispQuirks_;
};

/* -----------------------------------------------------------------------------
 * Camera Data
 */

AtomispCameraData::AtomispCameraData(AtomispPipelineHandler *pipe,
				   unsigned int numStreams,
				   MediaEntity *sensor)
	: Camera::Private(pipe), streams_(numStreams), rawStream_(nullptr)
{
	/*
	 * Find the shortest path from the camera sensor to a video capture
	 * device using the breadth-first search algorithm. This heuristic will
	 * be most likely to skip paths that aren't suitable for the simple
	 * pipeline handler on more complex devices, and is guaranteed to
	 * produce a valid path on all devices that have a single option.
	 *
	 * For instance, on the IPU-based i.MX6Q, the shortest path will skip
	 * encoders and image converters, and will end in a CSI capture device.
	 */
	std::unordered_set<MediaEntity *> visited;
	std::queue<std::tuple<MediaEntity *, MediaPad *>> queue;

	/* Remember at each entity where we came from. */
	std::unordered_map<MediaEntity *, Entity> parents;
	MediaEntity *entity = nullptr;
	MediaEntity *video = nullptr;
	MediaPad *sinkPad;

	queue.push({ sensor, nullptr });

	while (!queue.empty()) {
		std::tie(entity, sinkPad) = queue.front();
		queue.pop();

		/* Found the capture device. */
		if (entity->function() == MEDIA_ENT_F_IO_V4L) {
			LOG(AtomispPipeline, Debug)
				<< "Found capture device " << entity->name();
			video = entity;
			break;
		}

		visited.insert(entity);

		/*
		 * Add direct downstream entities to the search queue. If the
		 * current entity supports the subdev internal routing API,
		 * restrict the search to downstream entities reachable through
		 * active routes.
		 */

		std::vector<const MediaPad *> pads;
		bool supportsRouting = false;

		if (sinkPad) {
			pads = routedSourcePads(sinkPad);
			if (!pads.empty())
				supportsRouting = true;
		}

		if (pads.empty()) {
			for (const MediaPad *pad : entity->pads()) {
				if (!(pad->flags() & MEDIA_PAD_FL_SOURCE))
					continue;
				pads.push_back(pad);
			}
		}

		for (const MediaPad *pad : pads) {
			for (MediaLink *link : pad->links()) {
				MediaEntity *next = link->sink()->entity();
				if (visited.find(next) == visited.end()) {
					queue.push({ next, link->sink() });

					Entity e{ entity, supportsRouting, sinkPad, pad, link };
					parents.insert({ next, e });
				}
			}
		}
	}

	if (!video)
		return;

	/*
	 * With the parents, we can follow back our way from the capture device
	 * to the sensor. Store all the entities in the pipeline, from the
	 * camera sensor to the video node, in entities_.
	 */
	entities_.push_front({ entity, false, sinkPad, nullptr, nullptr });

	for (auto it = parents.find(entity); it != parents.end();
	     it = parents.find(entity)) {
		const Entity &e = it->second;
		entities_.push_front(e);
		entity = e.entity;
	}

	/* Finally also remember the sensor. */
	sensor_ = CameraSensorFactoryBase::create(sensor);
	if (!sensor_)
		return;

	const CameraSensorProperties::SensorDelays &delays = sensor_->sensorDelays();
	std::unordered_map<uint32_t, DelayedControls::ControlParams> params = {
		{ V4L2_CID_ANALOGUE_GAIN, { delays.gainDelay, false } },
		{ V4L2_CID_EXPOSURE, { delays.exposureDelay, false } },
		{ V4L2_CID_VBLANK, { 0, false } },
	};
	delayedCtrls_ = std::make_unique<DelayedControls>(sensor_->device(), params);

	/*
	 * The AtomISP is the pipeline master; the sensor subdev does not
	 * generate frame-start events in this configuration.  Force direct
	 * control application so AeLoop changes take effect immediately.
	 */
	frameStartEmitter_ = nullptr;
}

AtomispPipelineHandler *AtomispCameraData::pipe()
{
	return static_cast<AtomispPipelineHandler *>(Camera::Private::pipe());
}

int AtomispCameraData::init()
{
	AtomispPipelineHandler *pipe = AtomispCameraData::pipe();
	int ret;

	/* Open the converter, if any. */
	std::shared_ptr<MediaDevice> converter = pipe->converter();
	if (converter) {
		converter_ = ConverterFactoryBase::create(converter);
		if (!converter_) {
			LOG(AtomispPipeline, Warning)
				<< "Failed to create converter, disabling format conversion";
			converter_.reset();
		} else {
			converter_->inputBufferReady.connect(this, &AtomispCameraData::conversionInputDone);
			converter_->outputBufferReady.connect(this, &AtomispCameraData::conversionOutputDone);
		}
	}

	/*
	 * Instantiate Soft ISP if this is enabled for the given driver and no converter is used.
	 */
	if (!converter_ && pipe->swIspEnabled()) {
		swIsp_ = std::make_unique<SoftwareIsp>(pipe, sensor_.get(), &controlInfo_);
		if (!swIsp_->isValid()) {
			LOG(AtomispPipeline, Warning)
				<< "Failed to create software ISP, disabling software debayering";
			swIsp_.reset();
		} else {
			swIsp_->inputBufferReady.connect(this, &AtomispCameraData::conversionInputDone);
			swIsp_->outputBufferReady.connect(this, &AtomispCameraData::conversionOutputDone);
			swIsp_->ispStatsReady.connect(this, &AtomispCameraData::ispStatsReady);
			swIsp_->metadataReady.connect(this, &AtomispCameraData::metadataReady);
			swIsp_->setSensorControls.connect(this, &AtomispCameraData::setSensorControls);
		}
	}

	video_ = pipe->video(entities_.back().entity);
	ASSERT(video_);

	/*
	 * Setup links first as some subdev drivers take active links into
	 * account to propagate TRY formats. Such is life :-(
	 */
	ret = setupLinks();
	if (ret < 0)
		return ret;

	/*
	 * Generate the list of possible pipeline configurations by trying each
	 * media bus format and size supported by the sensor.
	 */
	for (unsigned int code : sensor_->mbusCodes()) {
		const std::vector<Size> sizes = sensor_->sizes(code);
		for (const Size &size : sizes)
			tryPipeline(code, size);

		/* AtomISP: no VGA fallback; 640x480 binning mode is known broken
		 * due to DVS padding mismatch (sensor provides 8px extra, binary
		 * needs 12px DVS envelope). Only >640x480 sizes are usable.
		 */
	}

	if (configs_.empty()) {
		LOG(AtomispPipeline, Error) << "No valid configuration found";
		return -EINVAL;
	}

	/* Map the pixel formats to configurations. */
	for (const Configuration &config : configs_) {
		formats_[config.captureFormat].push_back(&config);

		for (PixelFormat fmt : config.outputFormats)
			formats_[fmt].push_back(&config);
	}

	properties_ = sensor_->properties();

	return 0;
}

/*
 * Generate a list of supported pipeline configurations for a sensor media bus
 * code and size.
 *
 * First propagate the media bus code and size through the pipeline from the
 * camera sensor to the video node. Then, query the video node for all supported
 * pixel formats compatible with the media bus code. For each pixel format, store
 * a full pipeline configuration in the configs_ vector.
 */
void AtomispCameraData::tryPipeline(unsigned int code, const Size &size)
{
	/*
	 * Propagate the format through the pipeline, and enumerate the
	 * corresponding possible V4L2 pixel formats on the video node.
	 */
	V4L2SubdeviceFormat format{};
	format.code = code;
	format.size = size;

	int ret = setupFormats(&format, V4L2Subdevice::TryFormat);
	if (ret < 0) {
		/* Pipeline configuration failed, skip this configuration. */
		format.code = code;
		format.size = size;
		LOG(AtomispPipeline, Debug)
			<< "Sensor format " << format
			<< " not supported for this pipeline";
		return;
	}

	V4L2VideoDevice::Formats videoFormats = video_->formats(format.code);
	if (videoFormats.empty() && pipe()->atomispQuirks() &&
	    !video_->caps().hasMediaController()) {
		LOG(AtomispPipeline, Warning)
			<< "Video node " << video_->deviceNode()
			<< " does not support media-bus code filtering, retrying format enumeration without code";
		videoFormats = video_->formats();
	}

	LOG(AtomispPipeline, Debug)
		<< "Adding configuration for " << format.size
		<< " in pixel formats [ "
		<< utils::join(videoFormats, ", ",
				       [](const auto &f) {
					       return f.first.toString();
				       })
		<< " ]";

	auto addConfigurations = [&](const auto &videoFormat) {
		PixelFormat pixelFormat = videoFormat.first.toPixelFormat(false);
		if (!pixelFormat) {
			LOG(AtomispPipeline, Debug)
				<< "Unsupported V4L2 pixel format "
				<< videoFormat.first.toString();

			return;
		}

		std::vector<Size> captureSizes = {
			pipe()->atomispQuirks() ? size : format.size,
		};

		for (const Size &captureSize : captureSizes) {
			Size actualCaptureSize = captureSize;
				/* AtomISP: captureSize here is the sensor output size.
				 * Binning mode (~648x488) must be skipped due to DVS
				 * padding mismatch; only full-res (~1296x976) is usable.
				 */
				if (pipe()->atomispQuirks() && captureSize.width < 1000) {
					LOG(AtomispPipeline, Debug)
						<< "Skipping AtomISP binning-mode config for "
						<< captureSize << "-" << videoFormat.first
						<< " (binning mode has DVS padding mismatch)";
					continue;
				}

				if (pipe()->atomispQuirks()) {
					V4L2DeviceFormat captureFormat;
					captureFormat.fourcc = videoFormat.first;
					captureFormat.size = captureSize;

					/* AtomISP: firmware top/left cropping (up to 12px) plus
					 * driver padding can shift the output by up to 16px per
					 * axis relative to the requested sensor size.
					 */
					if (video_->tryFormat(&captureFormat) < 0 ||
					    (std::abs(static_cast<int>(captureFormat.size.width) -
						     static_cast<int>(captureSize.width)) > 16) ||
					    (std::abs(static_cast<int>(captureFormat.size.height) -
						     static_cast<int>(captureSize.height)) > 16)) {
						LOG(AtomispPipeline, Debug)
							<< "Skipping AtomISP configuration for "
							<< captureSize << "-" << videoFormat.first
							<< " (video node reports " << captureFormat << ")";
						continue;
					}

					/*
					 * Record the actual ISP output size (may include border
					 * padding) so validate() and the SPA plugin advertise the
					 * real buffer dimensions and avoid a stride mismatch.
					 */
					actualCaptureSize = captureFormat.size;
				}

			Configuration config;
			config.code = code;
			config.sensorSize = size;
			config.captureFormat = pixelFormat;
			config.captureSize = actualCaptureSize;

			if (converter_) {
				config.outputFormats = converter_->formats(pixelFormat);
				config.outputSizes = converter_->sizes(actualCaptureSize);
			} else if (swIsp_) {
				config.outputFormats = swIsp_->formats(pixelFormat);
				config.outputSizes = swIsp_->sizes(pixelFormat, actualCaptureSize);
				if (config.outputFormats.empty()) {
					/* Do not use swIsp for unsupported pixelFormat's. */
					config.outputFormats = { pixelFormat };
					config.outputSizes = config.captureSize;
				}
			} else {
				config.outputFormats = { pixelFormat };
				config.outputSizes = config.captureSize;
			}

			configs_.push_back(config);
		}
	};

	if (pipe()->atomispQuirks()) {
		static const std::array<uint32_t, 9> preferredFormats = {
			V4L2_PIX_FMT_UYVY,
			V4L2_PIX_FMT_YUYV,
			V4L2_PIX_FMT_NV12,
			V4L2_PIX_FMT_NV21,
			V4L2_PIX_FMT_YUV420,
			V4L2_PIX_FMT_YVU420,
			V4L2_PIX_FMT_YUV422P,
			V4L2_PIX_FMT_YUV444,
			V4L2_PIX_FMT_NV16,
		};

		std::vector<V4L2PixelFormat> seenFormats;
		seenFormats.reserve(videoFormats.size());

		for (uint32_t preferredFormat : preferredFormats) {
			auto it = videoFormats.find(V4L2PixelFormat(preferredFormat));
			if (it == videoFormats.end())
				continue;

			addConfigurations(*it);
			seenFormats.push_back(V4L2PixelFormat(preferredFormat));
		}

		for (const auto &videoFormat : videoFormats) {
			if (std::find(seenFormats.begin(), seenFormats.end(), videoFormat.first) !=
			    seenFormats.end())
				continue;

			addConfigurations(videoFormat);
		}
	} else {
		for (const auto &videoFormat : videoFormats)
			addConfigurations(videoFormat);
	}
}

int AtomispCameraData::setupLinks()
{
	int ret;

	/*
	 * Configure all links along the pipeline. Some entities may not allow
	 * multiple sink links to be enabled together, even on different sink
	 * pads. We must thus start by disabling all sink links (but the one we
	 * want to enable) before enabling the pipeline link.
	 *
	 * The entities_ list stores entities along with their source link. We
	 * need to process the link in the context of the sink entity, so
	 * record the source link of the current entity as the sink link of the
	 * next entity, and skip the first entity in the loop.
	 */
	MediaLink *sinkLink = nullptr;

	for (AtomispCameraData::Entity &e : entities_) {
		if (!sinkLink) {
			sinkLink = e.sourceLink;
			continue;
		}

		for (MediaPad *pad : e.entity->pads()) {
			/*
			 * If the entity supports the V4L2 internal routing API,
			 * assume that it may carry multiple independent streams
			 * concurrently, and only disable links on the sink and
			 * source pads used by the pipeline.
			 */
			if (e.supportsRouting && pad != e.sink && pad != e.source)
				continue;

			for (MediaLink *link : pad->links()) {
				if (link == sinkLink)
					continue;

				if ((link->flags() & MEDIA_LNK_FL_ENABLED) &&
				    !(link->flags() & MEDIA_LNK_FL_IMMUTABLE)) {
					ret = link->setEnabled(false);
					if (ret < 0)
						return ret;
				}
			}
		}

		if (!(sinkLink->flags() & MEDIA_LNK_FL_ENABLED)) {
			ret = sinkLink->setEnabled(true);
			if (ret < 0)
				return ret;
		}

		sinkLink = e.sourceLink;
	}

	return 0;
}

int AtomispCameraData::setupFormats(V4L2SubdeviceFormat *format,
				   V4L2Subdevice::Whence whence,
				   Transform transform)
{
	AtomispPipelineHandler *pipe = AtomispCameraData::pipe();
	int ret;

	/*
	 * Configure the format on the sensor output and propagate it through
	 * the pipeline.
	 */
	ret = sensor_->setFormat(format, transform);
	if (ret < 0)
		return ret;

	for (const Entity &e : entities_) {
		if (!e.sourceLink)
			break;

		MediaLink *link = e.sourceLink;
		MediaPad *source = link->source();
		MediaPad *sink = link->sink();

		if (source->entity() != sensor_->entity()) {
			V4L2Subdevice *subdev = pipe->subdev(source->entity());
			ret = subdev->getFormat(source->index(), format, whence);
			if (ret < 0)
				return ret;
		}

		if (sink->entity()->function() != MEDIA_ENT_F_IO_V4L) {
			V4L2SubdeviceFormat sourceFormat = *format;

			V4L2Subdevice *subdev = pipe->subdev(sink->entity());
			ret = subdev->setFormat(sink->index(), format, whence);
			if (ret < 0)
				return ret;

			if (format->code != sourceFormat.code ||
			    format->size != sourceFormat.size) {
					if (pipe->atomispQuirks() &&
					    format->code == sourceFormat.code) {
						int dw = std::abs(static_cast<int>(format->size.width) -
								  static_cast<int>(sourceFormat.size.width));
						int dh = std::abs(static_cast<int>(format->size.height) -
								  static_cast<int>(sourceFormat.size.height));

						if (dw <= 8 && dh <= 8) {
							LOG(AtomispPipeline, Debug)
								<< "Tolerating AtomISP source/sink size delta on "
								<< source->entity()->name() << ":" << source->index()
								<< " -> " << sink->entity()->name() << ":" << sink->index()
								<< " (source " << sourceFormat.size
								<< ", sink " << format->size << ")";
						} else {
							LOG(AtomispPipeline, Debug)
								<< "Source '" << source->entity()->name()
								<< "':" << source->index()
								<< " produces " << sourceFormat
								<< ", sink '" << sink->entity()->name()
								<< "':" << sink->index()
								<< " requires " << *format;
							return -EINVAL;
						}
					} else {
						LOG(AtomispPipeline, Debug)
							<< "Source '" << source->entity()->name()
							<< "':" << source->index()
							<< " produces " << sourceFormat
							<< ", sink '" << sink->entity()->name()
							<< "':" << sink->index()
							<< " requires " << *format;
						return -EINVAL;
					}
			}
		}

		LOG(AtomispPipeline, Debug)
			<< "Link " << *link << ": configured with format "
			<< *format;
	}

	return 0;
}

void AtomispCameraData::imageBufferReady(FrameBuffer *buffer)
{
	AtomispPipelineHandler *pipe = AtomispCameraData::pipe();

	/*
	 * If an error occurred during capture, or if the buffer was cancelled,
	 * complete the request, even if the converter is in use as there's no
	 * point converting an erroneous buffer.
	 */
	if (buffer->metadata().status != FrameMetadata::FrameSuccess) {
		if (!useConversion_ || rawStream_) {
			/* No conversion, just complete the request. */
			Request *request = buffer->request();
			pipe->completeBuffer(request, buffer);
			AtomispFrameInfo *info = frameInfo_.find(request->sequence());
			if (info)
				info->metadataRequired = false;
			tryCompleteRequest(request);
			return;
		}

		/*
		 * The converter or Software ISP is in use. Requeue the internal
		 * buffer for capture (unless the stream is being stopped), and
		 * complete the request with all the user-facing buffers.
		 */
		if (buffer->metadata().status != FrameMetadata::FrameCancelled)
			video_->queueBuffer(buffer);

		if (conversionQueue_.empty())
			return;

		const RequestOutputs &outputs = conversionQueue_.front();
		for (auto &[stream, buf] : outputs.outputs)
			pipe->completeBuffer(outputs.request, buf);
		AtomispFrameInfo *info = frameInfo_.find(outputs.request->sequence());
		if (info)
			info->metadataRequired = false;
		tryCompleteRequest(outputs.request);
		conversionQueue_.pop();

		return;
	}

	/*
	 * Record the sensor's timestamp in the request metadata. The request
	 * needs to be obtained from the user-facing buffer, as internal
	 * buffers are free-wheeling and have no request associated with them.
	 *
	 * \todo The sensor timestamp should be better estimated by connecting
	 * to the V4L2Device::frameStart signal if the platform provides it.
	 */
	Request *request = buffer->request();

	if (useConversion_ && !conversionQueue_.empty()) {
		const std::map<const Stream *, FrameBuffer *> &outputs =
			conversionQueue_.front().outputs;
		if (!outputs.empty()) {
			FrameBuffer *outputBuffer = outputs.begin()->second;
			if (outputBuffer)
				request = outputBuffer->request();
		}
	}

	if (request)
		request->_d()->metadata().set(controls::SensorTimestamp,
					      buffer->metadata().timestamp);

	/* Sample luminance for AtomISP AE on every successful capture buffer */
	if (atomispAe_)
		atomispAe_->processBuffer(buffer->metadata().sequence, buffer);

	/*
	 * Queue the captured and the request buffer to the converter or Software
	 * ISP if format conversion is needed. If there's no queued request, just
	 * requeue the captured buffer for capture.
	 */
	if (useConversion_) {
		if (conversionQueue_.empty()) {
			if (!rawStream_)
				video_->queueBuffer(buffer);
			return;
		}

		if (converter_)
			converter_->queueBuffers(buffer, conversionQueue_.front().outputs);
		else
			/*
			 * request->sequence() cannot be retrieved from `buffer' inside
			 * queueBuffers because unique_ptr's make buffer->request() invalid
			 * already here.
			 */
			swIsp_->queueBuffers(request->sequence(), buffer,
					     conversionQueue_.front().outputs);

		conversionQueue_.pop();
		return;
	}

	/* Otherwise simply complete the request. */
	pipe->completeBuffer(request, buffer);
	tryCompleteRequest(request);
}

void AtomispCameraData::clearIncompleteRequests()
{
	while (!conversionQueue_.empty()) {
		pipe()->cancelRequest(conversionQueue_.front().request);
		conversionQueue_.pop();
	}
}

void AtomispCameraData::tryCompleteRequest(Request *request)
{
	if (request->hasPendingBuffers())
		return;

	AtomispFrameInfo *info = frameInfo_.find(request->sequence());
	if (!info) {
		/* Something is really wrong, let's return. */
		return;
	}

	if (info->metadataRequired && !info->metadataProcessed)
		return;

	frameInfo_.destroy(info->frame);
	pipe()->completeRequest(request);
}

void AtomispCameraData::conversionInputDone(FrameBuffer *buffer)
{
	if (rawStream_) {
		/* Complete the input buffer as with raw-only processing. */
		Request *request = buffer->request();
		if (pipe()->completeBuffer(request, buffer))
			tryCompleteRequest(request);
	} else {
		/* Queue the input buffer back for capture. */
		video_->queueBuffer(buffer);
	}
}

void AtomispCameraData::conversionOutputDone(FrameBuffer *buffer)
{
	AtomispPipelineHandler *pipe = AtomispCameraData::pipe();

	/* Complete the buffer and the request. */
	Request *request = buffer->request();
	if (pipe->completeBuffer(request, buffer))
		tryCompleteRequest(request);
}

void AtomispCameraData::ispStatsReady(uint32_t frame, uint32_t bufferId)
{
	swIsp_->processStats(frame, bufferId,
			     delayedCtrls_->get(frame));
}

void AtomispCameraData::metadataReady(uint32_t frame, const ControlList &metadata)
{
	AtomispFrameInfo *info = frameInfo_.find(frame);
	if (!info)
		return;

	info->request->_d()->metadata().merge(metadata);
	info->metadataProcessed = true;
	tryCompleteRequest(info->request);
}

void AtomispCameraData::setSensorControls(const ControlList &sensorControls)
{
	delayedCtrls_->push(sensorControls);
	/*
	 * Directly apply controls now if there is no frameStart signal.
	 *
	 * \todo Applying controls directly not only increases the risk of
	 * applying them to the wrong frame (or across a frame boundary),
	 * but it also bypasses delayedCtrls_, creating AGC regulation issues.
	 * Both problems should be fixed.
	 */
	if (!frameStartEmitter_) {
		ControlList ctrls(sensorControls);
		int ret = sensor_->setControls(&ctrls);
		if (ret) {
			LOG(AtomispPipeline, Warning)
				<< "AtomISP AE: batched sensor controls failed: " << ret
				<< ", falling back to per-control writes";

			for (const auto &[id, value] : sensorControls) {
				ControlList oneCtrl(sensor_->controls());
				oneCtrl.set(id, value);
				ret = sensor_->setControls(&oneCtrl);
				if (ret)
					LOG(AtomispPipeline, Warning)
						<< "AtomISP AE: failed to apply control "
						<< utils::hex(id) << ": " << ret;
			}
		}
	}
}

/* Retrieve all source pads connected to a sink pad through active routes. */
std::vector<const MediaPad *> AtomispCameraData::routedSourcePads(MediaPad *sink)
{
	MediaEntity *entity = sink->entity();
	std::unique_ptr<V4L2Subdevice> subdev =
		std::make_unique<V4L2Subdevice>(entity);

	int ret = subdev->open();
	if (ret < 0)
		return {};

	V4L2Subdevice::Routing routing = {};
	ret = subdev->getRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret < 0)
		return {};

	std::vector<const MediaPad *> pads;

	for (const V4L2Subdevice::Route &route : routing) {
		if (sink->index() != route.sink.pad ||
		    !(route.flags & V4L2_SUBDEV_ROUTE_FL_ACTIVE))
			continue;

		const MediaPad *pad = entity->getPadByIndex(route.source.pad);
		if (!pad) {
			LOG(AtomispPipeline, Warning)
				<< "Entity " << entity->name()
				<< " has invalid route source pad "
				<< route.source.pad;
		}

		pads.push_back(pad);
	}

	return pads;
}

/* -----------------------------------------------------------------------------
 * Camera Configuration
 */

AtomispCameraConfiguration::AtomispCameraConfiguration(Camera *camera,
						     AtomispCameraData *data)
	: CameraConfiguration(), camera_(camera->shared_from_this()),
	  data_(data), pipeConfig_(nullptr)
{
}

namespace {

static Size adjustSize(const Size &requestedSize, const SizeRange &supportedSizes)
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

} /* namespace */

CameraConfiguration::Status AtomispCameraConfiguration::validate()
{
	const CameraSensor *sensor = data_->sensor_.get();
	Status status = Valid;

	if (config_.empty())
		return Invalid;

	Orientation requestedOrientation = orientation;
	if (data_->pipe()->atomispQuirks()) {
		/*
		 * Use the sensor's native transform so firmware-programmed flips
		 * (e.g. 180° mounted sensors) are honoured. Guard against
		 * axis-transposing transforms (90°/270°) that would swap the
		 * landscape sensor into a portrait capture path.
		 */
		combinedTransform_ = sensor->computeTransform(&orientation);
		if (!!(combinedTransform_ & Transform::Transpose)) {
			combinedTransform_ = Transform::Identity;
			orientation = requestedOrientation;
		} else if (orientation != requestedOrientation) {
			status = Adjusted;
		}
	} else {
		combinedTransform_ = sensor->computeTransform(&orientation);
		if (orientation != requestedOrientation)
			status = Adjusted;
	}

	/* Cap the number of entries to the available streams. */
	if (config_.size() > data_->streams_.size()) {
		config_.resize(data_->streams_.size());
		status = Adjusted;
	}

	/* Find the largest stream sizes. */
	Size maxProcessedStreamSize;
	Size maxRawStreamSize;
	for (const StreamConfiguration &cfg : config_) {
		if (isRaw(cfg))
			maxRawStreamSize.expandTo(cfg.size);
		else
			maxProcessedStreamSize.expandTo(cfg.size);
	}

	LOG(AtomispPipeline, Debug)
		<< "Largest processed stream size is " << maxProcessedStreamSize;
	LOG(AtomispPipeline, Debug)
		<< "Largest raw stream size is " << maxRawStreamSize;

	/* Cap the number of raw stream configurations */
	unsigned int rawCount = 0;
	PixelFormat requestedRawFormat;
	for (const StreamConfiguration &cfg : config_) {
		if (!isRaw(cfg))
			continue;
		requestedRawFormat = cfg.pixelFormat;
		rawCount++;
	}

	if (rawCount > 1) {
		LOG(AtomispPipeline, Error)
			<< "Camera configuration with multiple raw streams not supported";
		return Invalid;
	}

	/*
	 * Find the best configuration for the pipeline using a heuristic.
	 * First select the pixel format based on the raw streams followed by
	 * non-raw streams (which are considered ordered from highest to lowest
	 * priority). Default to the first pipeline configuration if no streams
	 * request a supported pixel format.
	 */
	const std::vector<const AtomispCameraData::Configuration *> *configs =
		&data_->formats_.begin()->second;

	auto rawIter = data_->formats_.find(requestedRawFormat);
	if (rawIter != data_->formats_.end()) {
		configs = &rawIter->second;
	} else {
		for (const StreamConfiguration &cfg : config_) {
			auto it = data_->formats_.find(cfg.pixelFormat);
			if (it != data_->formats_.end()) {
				configs = &it->second;
				break;
			}
		}
	}

	/*
	 * \todo Pick the best sensor output media bus format when the
	 * requested pixel format can be produced from multiple sensor media
	 * bus formats.
	 */

	/*
	 * Then pick, among the possible configuration for the pixel format,
	 * the smallest sensor resolution that can accommodate all streams
	 * without upscaling.
	 */
	const AtomispCameraData::Configuration *maxPipeConfig = nullptr;
	const AtomispCameraData::Configuration *maxPipeConfigNonRaw = nullptr;
	pipeConfig_ = nullptr;
	const bool requireNonRawCapture =
		data_->pipe()->atomispQuirks() && maxRawStreamSize.isNull();
	auto atomispCaptureFormatScore = [](PixelFormat format) {
		/* Prefer packed interleaved YUV; accept any non-raw YUV format */
		if (format == PixelFormat{ V4L2_PIX_FMT_UYVY }) return 0;
		if (format == PixelFormat{ V4L2_PIX_FMT_YUYV }) return 1;
		if (format == PixelFormat{ V4L2_PIX_FMT_NV12 }) return 2;
		if (format == PixelFormat{ V4L2_PIX_FMT_NV21 }) return 3;
		if (format == PixelFormat{ V4L2_PIX_FMT_YUV420 }) return 4;
		if (format == PixelFormat{ V4L2_PIX_FMT_YVU420 }) return 5;
		if (format == PixelFormat{ V4L2_PIX_FMT_NV16 }) return 6;
		if (!BayerFormat::fromPixelFormat(format).isValid()) return 7;
		return -1;
	};

	for (const AtomispCameraData::Configuration *pipeConfig : *configs) {
		const Size &captureSize = pipeConfig->captureSize;
		const Size &maxOutputSize = pipeConfig->outputSizes.max;
		const bool captureIsRaw =
			BayerFormat::fromPixelFormat(pipeConfig->captureFormat).isValid();
		const int captureFormatScore =
			atomispCaptureFormatScore(pipeConfig->captureFormat);

		if (!captureIsRaw && (!requireNonRawCapture || captureFormatScore >= 0) &&
		    (!maxPipeConfigNonRaw || maxPipeConfigNonRaw->captureSize < captureSize))
			maxPipeConfigNonRaw = pipeConfig;

		if (requireNonRawCapture && (captureIsRaw || captureFormatScore < 0))
			continue;

		if (maxOutputSize.width >= maxProcessedStreamSize.width &&
		    maxOutputSize.height >= maxProcessedStreamSize.height &&
		    captureSize.width >= maxRawStreamSize.width &&
		    captureSize.height >= maxRawStreamSize.height) {
			if (!pipeConfig_ || captureSize < pipeConfig_->captureSize)
				pipeConfig_ = pipeConfig;
		}

		if (!maxPipeConfig || maxPipeConfig->captureSize < captureSize)
			maxPipeConfig = pipeConfig;
	}

	/* If no configuration was large enough, select the largest one. */
	if (!pipeConfig_) {
		if (requireNonRawCapture && maxPipeConfigNonRaw)
			pipeConfig_ = maxPipeConfigNonRaw;
		else
			pipeConfig_ = maxPipeConfig;
	}

	if (!pipeConfig_) {
		LOG(AtomispPipeline, Error) << "No valid pipeline configuration found";
		return Invalid;
	}

	LOG(AtomispPipeline, Debug)
		<< "Picked "
		<< V4L2SubdeviceFormat{ pipeConfig_->code, pipeConfig_->sensorSize, {} }
		<< " -> " << pipeConfig_->captureSize
		<< "-" << pipeConfig_->captureFormat
		<< " for max processed stream size " << maxProcessedStreamSize
		<< " and max raw stream size " << maxRawStreamSize;

	/*
	 * Adjust the requested streams.
	 *
	 * Enable usage of the converter when producing multiple streams, as
	 * the video capture device can't capture to multiple buffers.
	 *
	 * It is possible to produce up to one stream without conversion
	 * (provided the format and size match), at the expense of more complex
	 * buffer handling (including allocation of internal buffers to be used
	 * when a request doesn't contain a buffer for the stream that doesn't
	 * require any conversion, similar to raw capture use cases). This is
	 * left as a future improvement.
	 */
	needConversion_ = config_.size() > 1 + rawCount;

	for (unsigned int i = 0; i < config_.size(); ++i) {
		StreamConfiguration &cfg = config_[i];
		const bool raw = isRaw(cfg);

		/* Adjust the pixel format and size. */
		if (raw) {
			if (cfg.pixelFormat != pipeConfig_->captureFormat ||
			    cfg.size != pipeConfig_->captureSize) {
				cfg.pixelFormat = pipeConfig_->captureFormat;
				cfg.size = pipeConfig_->captureSize;

				LOG(AtomispPipeline, Debug)
					<< "Adjusting raw stream to "
					<< cfg.toString();
				status = Adjusted;
			}
		} else {
			if (pipeConfig_->outputFormats.empty())
				return Invalid;

			auto it = std::find(pipeConfig_->outputFormats.begin(),
					    pipeConfig_->outputFormats.end(),
					    cfg.pixelFormat);
			if (it == pipeConfig_->outputFormats.end())
				it = pipeConfig_->outputFormats.begin();

			PixelFormat pixelFormat = *it;
			if (cfg.pixelFormat != pixelFormat) {
				LOG(AtomispPipeline, Debug)
					<< "Adjusting processed pixel format from "
					<< cfg.pixelFormat << " to " << pixelFormat;
				cfg.pixelFormat = pixelFormat;
				status = Adjusted;
			}
		}

		/*
		 * Best effort to fix the color space. If the color space is not set,
		 * set it according to the pixel format, which may not be correct (pixel
		 * formats and color spaces are different things, although somewhat
		 * related) but we don't have a better option at the moment. Then in any
		 * case, perform the standard pixel format based color space adjustment.
		 */
		if (!cfg.colorSpace) {
			const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
			switch (info.colourEncoding) {
			case PixelFormatInfo::ColourEncodingRGB:
				cfg.colorSpace = ColorSpace::Srgb;
				break;
			case PixelFormatInfo::ColourEncodingYUV:
				cfg.colorSpace = ColorSpace::Sycc;
				break;
			default:
				cfg.colorSpace = ColorSpace::Raw;
			}
			/*
			 * Adjust the assigned color space to make sure everything is OK.
			 * Since this is assigning an unspecified color space rather than
			 * adjusting a requested one, changes here shouldn't set the status
			 * to Adjusted.
			 */
			cfg.colorSpace->adjust(cfg.pixelFormat);
			LOG(AtomispPipeline, Debug)
				<< "Unspecified color space set to "
				<< cfg.colorSpace.value().toString();
		} else {
			if (cfg.colorSpace->adjust(cfg.pixelFormat)) {
				LOG(AtomispPipeline, Debug)
					<< "Color space adjusted to "
					<< cfg.colorSpace.value().toString();
				status = Adjusted;
			}
		}

		if (!raw && !pipeConfig_->outputSizes.contains(cfg.size)) {
			Size adjustedSize = pipeConfig_->captureSize;
			/*
			 * The converter (when present) may not be able to output
			 * a size identical to its input size. The capture size is thus
			 * not guaranteed to be a valid output size. In such cases, use
			 * the smaller valid output size closest to the requested.
			 */
			if (!pipeConfig_->outputSizes.contains(adjustedSize))
				adjustedSize = adjustSize(cfg.size, pipeConfig_->outputSizes);
			LOG(AtomispPipeline, Debug)
				<< "Adjusting size from " << cfg.size
				<< " to " << adjustedSize;
			cfg.size = adjustedSize;
			status = Adjusted;
		}

		/* \todo Create a libcamera core class to group format and size */
		if (cfg.pixelFormat != pipeConfig_->captureFormat ||
		    cfg.size != pipeConfig_->captureSize)
			needConversion_ = true;

		/* Set the stride and frameSize. */
		if (needConversion_ && !raw) {
			std::tie(cfg.stride, cfg.frameSize) =
				data_->converter_
					? data_->converter_->strideAndFrameSize(cfg.pixelFormat,
										cfg.size)
					: data_->swIsp_->strideAndFrameSize(cfg.pixelFormat,
									    cfg.size);
			if (cfg.stride == 0)
				return Invalid;
		} else {
			V4L2DeviceFormat format;
			format.fourcc = data_->video_->toV4L2PixelFormat(cfg.pixelFormat);
			format.size = cfg.size;

			int ret = data_->video_->tryFormat(&format);
			if (ret < 0)
				return Invalid;

			cfg.stride = format.planes[0].bpl;
			cfg.frameSize = format.planes[0].size;
		}

		const unsigned int bufferCount = cfg.bufferCount;
		if (!bufferCount)
			cfg.bufferCount = kNumBuffersDefault;
		else if (bufferCount > kNumBuffersMax)
			cfg.bufferCount = kNumBuffersMax;

		if (cfg.bufferCount != bufferCount) {
			LOG(AtomispPipeline, Debug)
				<< "Adjusting bufferCount from " << bufferCount
				<< " to " << cfg.bufferCount;
			status = Adjusted;
		}
	}

	return status;
}

/* -----------------------------------------------------------------------------
 * Pipeline Handler
 */

AtomispPipelineHandler::AtomispPipelineHandler(CameraManager *manager)
	: PipelineHandler(manager, kMaxQueuedRequestsDevice),
	  converter_(nullptr),
	  swIspEnabled_(false),
	  atomispQuirks_(true)
{
}

std::unique_ptr<CameraConfiguration>
AtomispPipelineHandler::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	AtomispCameraData *data = cameraData(camera);
	std::unique_ptr<CameraConfiguration> config =
		std::make_unique<AtomispCameraConfiguration>(camera, data);

	if (roles.empty())
		return config;

	bool processedRequested = false;
	bool rawRequested = false;
	for (const auto &role : roles)
		if (role == StreamRole::Raw) {
			if (rawRequested) {
				LOG(AtomispPipeline, Error)
					<< "Can't capture multiple raw streams";
				return nullptr;
			}
			rawRequested = true;
		} else {
			processedRequested = true;
		}

	/* Create the formats maps. */
	std::map<PixelFormat, std::vector<SizeRange>> processedFormats;
	std::map<PixelFormat, std::vector<SizeRange>> rawFormats;

	for (const AtomispCameraData::Configuration &cfg : data->configs_) {
		rawFormats[cfg.captureFormat].push_back(cfg.captureSize);
		for (PixelFormat format : cfg.outputFormats)
			processedFormats[format].push_back(cfg.outputSizes);
	}

	if (processedRequested && processedFormats.empty()) {
		LOG(AtomispPipeline, Error)
			<< "Processed stream requested but no corresponding output configuration found";
		return nullptr;
	}
	if (rawRequested && rawFormats.empty()) {
		LOG(AtomispPipeline, Error)
			<< "Raw stream requested but no corresponding output configuration found";
		return nullptr;
	}

	auto setUpFormatSizes = [](std::map<PixelFormat, std::vector<SizeRange>> &formats) {
		/* Sort the sizes and merge any consecutive overlapping ranges. */

		for (auto &[format, sizes] : formats) {
			std::sort(sizes.begin(), sizes.end(),
				  [](SizeRange &a, SizeRange &b) {
					  return a.min < b.min;
				  });

			auto cur = sizes.begin();
			auto next = cur;

			while (++next != sizes.end()) {
				if (cur->max.width >= next->min.width &&
				    cur->max.height >= next->min.height)
					cur->max = next->max;
				else if (++cur != next)
					*cur = *next;
			}

			sizes.erase(++cur, sizes.end());
		}
	};
	setUpFormatSizes(processedFormats);
	setUpFormatSizes(rawFormats);

	/*
	 * Create the stream configurations. Take the first entry in the formats
	 * map as the default, for lack of a better option.
	 *
	 * \todo Implement a better way to pick the default format
	 */
		auto pickDefaultFormat = [&](const auto &formats, bool processed) {
			if (data->pipe()->atomispQuirks() && processed) {
			static const std::array<PixelFormat, 9> preferredFormats = {
				PixelFormat{ V4L2_PIX_FMT_UYVY },
				PixelFormat{ V4L2_PIX_FMT_YUYV },
				PixelFormat{ V4L2_PIX_FMT_NV12 },
				PixelFormat{ V4L2_PIX_FMT_NV21 },
				PixelFormat{ V4L2_PIX_FMT_YUV420 },
				PixelFormat{ V4L2_PIX_FMT_YVU420 },
				PixelFormat{ V4L2_PIX_FMT_YUV422P },
				PixelFormat{ V4L2_PIX_FMT_YUV444 },
				PixelFormat{ V4L2_PIX_FMT_NV16 },
			};

			for (const PixelFormat &preferredFormat : preferredFormats) {
				auto it = formats.find(preferredFormat);
				if (it != formats.end())
					return it->first;
			}
		}

		return formats.begin()->first;
	};

	for (StreamRole role : roles) {
		const auto &formats = (role == StreamRole::Raw ? rawFormats : processedFormats);
		StreamConfiguration cfg{ StreamFormats{ formats } };
		cfg.pixelFormat = pickDefaultFormat(formats, role != StreamRole::Raw);
		cfg.size = formats.begin()->second[0].max;

		/*
		 * Pre-compute stride with AtomISP's 64-byte row alignment so the
		 * PipeWire SPA plugin uses the correct value during format
		 * negotiation (before configure() can update it).
		 */
		if (atomispQuirks_ && role != StreamRole::Raw) {
			const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
			if (info.isValid()) {
				cfg.stride = ((info.stride(cfg.size.width, 0, 1) + 63) / 64) * 64;
				cfg.frameSize = cfg.stride * cfg.size.height;
			}
		}

		config->addConfiguration(cfg);
	}

	config->validate();

	return config;
}

int AtomispPipelineHandler::configure(Camera *camera, CameraConfiguration *c)
{
	AtomispCameraConfiguration *config =
		static_cast<AtomispCameraConfiguration *>(c);
	AtomispCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	int ret;

	/*
	 * Configure links on the pipeline and propagate formats from the
	 * sensor to the video node.
	 */
	ret = data->setupLinks();
	if (ret < 0)
		return ret;

	const AtomispCameraData::Configuration *pipeConfig = config->pipeConfig();
	V4L2SubdeviceFormat format{};
	format.code = pipeConfig->code;
	format.size = pipeConfig->sensorSize;

	ret = data->setupFormats(&format, V4L2Subdevice::ActiveFormat,
				 config->combinedTransform());
	if (ret < 0)
		return ret;

	/* Configure the video node, taking into account any Bayer pattern change. */
	V4L2PixelFormat videoFormat = video->toV4L2PixelFormat(pipeConfig->captureFormat);

	/*
	 * Only recalculate the V4L2 pixel format from Bayer order if the capture
	 * format is itself a Bayer format. Some pipelines legitimately change media
	 * bus codes while keeping a non-Bayer capture node format.
	 */
	if (format.code != pipeConfig->code &&
	    BayerFormat::fromPixelFormat(pipeConfig->captureFormat).isValid()) {
		BayerFormat cfgBayer = BayerFormat::fromPixelFormat(pipeConfig->captureFormat);
		if (cfgBayer.isValid()) {
			cfgBayer.order = data->sensor_->bayerOrder(config->combinedTransform());
			V4L2PixelFormat transformedFormat = cfgBayer.toV4L2PixelFormat();
			if (transformedFormat.isValid())
				videoFormat = transformedFormat;
		}
	}

	Size captureSize = pipeConfig->captureSize;
	bool atomispSizeAdjusted = false;
	V4L2DeviceFormat captureFormat;
	captureFormat.fourcc = videoFormat;
	captureFormat.size = captureSize;

	ret = video->setFormat(&captureFormat);
	if (ret)
		return ret;

	if (captureFormat.planesCount != 1) {
		LOG(AtomispPipeline, Error)
			<< "Planar formats using non-contiguous memory not supported";
		return -EINVAL;
	}

	if (captureFormat.fourcc != videoFormat) {
		LOG(AtomispPipeline, Error)
			<< "Unable to configure capture in "
			<< captureSize << "-" << videoFormat
			<< " (got " << captureFormat << ")";
		return -EINVAL;
	}

	if (captureFormat.size != captureSize) {
		if (data->pipe()->atomispQuirks() &&
		    std::abs(static_cast<int>(captureFormat.size.width) -
			     static_cast<int>(captureSize.width)) <= 16 &&
		    std::abs(static_cast<int>(captureFormat.size.height) -
			     static_cast<int>(captureSize.height)) <= 16) {
			LOG(AtomispPipeline, Debug)
				<< "Tolerating AtomISP capture size delta: requested "
				<< captureSize << ", got " << captureFormat.size;
			captureSize = captureFormat.size;
			atomispSizeAdjusted = true;
		} else {
			LOG(AtomispPipeline, Error)
				<< "Unable to configure capture in "
				<< captureSize << "-" << videoFormat
				<< " (got " << captureFormat << ")";
			return -EINVAL;
		}
	}

	/* Configure the converter if needed. */
	std::vector<std::reference_wrapper<const StreamConfiguration>> outputCfgs;
	data->useConversion_ = config->needConversion();

	data->rawStream_ = nullptr;
	for (unsigned int i = 0; i < config->size(); ++i) {
		StreamConfiguration &cfg = config->at(i);
		bool rawStream = isRaw(cfg);

		cfg.setStream(&data->streams_[i]);

		/* AtomISP: always update stride and frame size from the actual
		 * hardware format — the ISP may add alignment padding (e.g.
		 * bpl=2624 for a 1296-wide UYVY frame) even without a size
		 * adjustment, and the SPA plugin must see the real values.
		 */
		if (!rawStream) {
			if (atomispSizeAdjusted) {
				LOG(AtomispPipeline, Debug)
					<< "Updating AtomISP stream config from " << cfg.size
					<< " to " << captureSize;
				cfg.size = captureSize;
			}
			cfg.stride = captureFormat.planes[0].bpl;
			cfg.frameSize = captureFormat.planes[0].size;
		}

		if (data->useConversion_ && !rawStream)
			outputCfgs.push_back(cfg);

		if (rawStream)
			data->rawStream_ = &data->streams_[i];
	}

	if (outputCfgs.empty())
		return 0;

	/* Configure the AtomISP luminance AE loop when applicable */
	if (atomispQuirks_) {
		PixelFormat capturePf = captureFormat.fourcc.toPixelFormat();
		data->atomispAe_ = std::make_unique<AtomispAeLoop>();
		if (!data->atomispAe_->configure(data->sensor_.get(),
						 capturePf, captureSize,
						 captureFormat.planes[0].bpl)) {
			LOG(AtomispPipeline, Warning) << "AtomISP AE loop disabled";
			data->atomispAe_.reset();
		} else {
			data->atomispAe_->setSensorControls.connect(
				data, &AtomispCameraData::setSensorControls);
			/* Write initial exposure/gain so hardware starts bright. */
			data->atomispAe_->bootstrap();
		}
	}

	StreamConfiguration inputCfg;
	inputCfg.pixelFormat = videoFormat.toPixelFormat();
	inputCfg.size = captureSize;
	inputCfg.stride = captureFormat.planes[0].bpl;
	inputCfg.bufferCount = kNumInternalBuffers;

	if (data->converter_) {
		return data->converter_->configure(inputCfg, outputCfgs);
	} else {
		ipa::soft::IPAConfigInfo configInfo;
		configInfo.sensorControls = data->sensor_->controls();
		return data->swIsp_->configure(inputCfg, outputCfgs, configInfo);
	}
}

int AtomispPipelineHandler::exportFrameBuffers(Camera *camera, Stream *stream,
					      std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	AtomispCameraData *data = cameraData(camera);
	unsigned int count = stream->configuration().bufferCount;

	/*
	 * Export buffers on the converter or capture video node, depending on
	 * whether the converter is used or not.
	 */
	if (data->useConversion_ && stream != data->rawStream_)
		return data->converter_
			       ? data->converter_->exportBuffers(stream, count, buffers)
			       : data->swIsp_->exportBuffers(stream, count, buffers);
	else
		return data->video_->exportBuffers(count, buffers);
}

int AtomispPipelineHandler::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	AtomispCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	V4L2Subdevice *frameStartEmitter = data->frameStartEmitter_;
	AtomispPipelineHandler *pipe = data->pipe();
	int ret;

	const MediaPad *pad = acquirePipeline(data);
	if (pad) {
		LOG(AtomispPipeline, Info)
			<< "Failed to acquire pipeline, entity "
			<< pad->entity()->name() << " in use";
		return -EBUSY;
	}

	if (data->useConversion_ && !data->rawStream_) {
		/*
		 * When using the converter allocate a fixed number of internal
		 * buffers.
		 */
		ret = video->allocateBuffers(kNumInternalBuffers,
					     &data->conversionBuffers_);
	} else {
		/*
		 * Otherwise, prepare for using buffers from either the raw stream, if
		 * requested, or the only stream configured.
		 */
		Stream *stream = (data->rawStream_ ? data->rawStream_ : &data->streams_[0]);
		ret = video->importBuffers(stream->configuration().bufferCount);
	}
	if (ret < 0) {
		releasePipeline(data);
		return ret;
	}

	video->bufferReady.connect(data, &AtomispCameraData::imageBufferReady);

	/* Keep sensor firmware AE disabled: AtomISP AE loop drives exposure/gain. */
	for (const AtomispCameraData::Entity &entity : data->entities_) {
		V4L2Subdevice *sd = pipe->subdev(entity.entity);
		if (!sd)
			continue;

		const ControlInfoMap &ctrls = sd->controls();
		auto it = ctrls.find(V4L2_CID_EXPOSURE_AUTO);
		if (it == ctrls.end())
			continue;

		ControlList ifpCtrls(ctrls);
		ifpCtrls.set(V4L2_CID_EXPOSURE_AUTO, V4L2_EXPOSURE_MANUAL);
		ret = sd->setControls(&ifpCtrls);
		if (ret)
			LOG(AtomispPipeline, Warning)
				<< "Failed to set exposure_auto=MANUAL on "
				<< sd->entity()->name() << ": " << ret;
		else
			LOG(AtomispPipeline, Debug)
				<< "Set exposure_auto=MANUAL on "
				<< sd->entity()->name();
		break;
	}

	data->delayedCtrls_->reset();
	if (frameStartEmitter) {
		ret = frameStartEmitter->setFrameStartEnabled(true);
		if (ret) {
			if (atomispQuirks_) {
				/*
				 * On AtomISP, frame-start enabling can fail at runtime despite
				 * the entity advertising support. Continue without frame-start
				 * driven delayed controls so streaming can still operate.
				 */
				LOG(AtomispPipeline, Warning)
					<< "Frame start events unavailable on "
					<< frameStartEmitter->entity()->name() << ": "
					<< strerror(-ret) << " (" << ret << "), continuing without them";
				frameStartEmitter = nullptr;
			} else {
				stop(camera);
				return ret;
			}
		} else {
			frameStartEmitter->frameStart.connect(data->delayedCtrls_.get(),
						      &DelayedControls::applyControls);
		}
	}

	ret = video->streamOn();
	if (ret < 0) {
		stop(camera);
		return ret;
	}

	if (data->useConversion_) {
		if (data->converter_)
			ret = data->converter_->start();
		else if (data->swIsp_)
			ret = data->swIsp_->start();
		else
			ret = 0;

		if (ret < 0) {
			stop(camera);
			return ret;
		}

		/* Queue all internal buffers for capture. */
		if (!data->rawStream_)
			for (std::unique_ptr<FrameBuffer> &buffer : data->conversionBuffers_)
				video->queueBuffer(buffer.get());
	}

	return 0;
}

void AtomispPipelineHandler::stopDevice(Camera *camera)
{
	AtomispCameraData *data = cameraData(camera);
	V4L2VideoDevice *video = data->video_;
	V4L2Subdevice *frameStartEmitter = data->frameStartEmitter_;

	if (frameStartEmitter) {
		frameStartEmitter->setFrameStartEnabled(false);
		frameStartEmitter->frameStart.disconnect(data->delayedCtrls_.get(),
							 &DelayedControls::applyControls);
	}

	if (data->useConversion_) {
		if (data->converter_)
			data->converter_->stop();
		else if (data->swIsp_)
			data->swIsp_->stop();
	}

	video->streamOff();
	video->releaseBuffers();

	video->bufferReady.disconnect(data, &AtomispCameraData::imageBufferReady);

	data->frameInfo_.clear();
	data->clearIncompleteRequests();
	data->conversionBuffers_.clear();

	releasePipeline(data);
}

int AtomispPipelineHandler::queueRequestDevice(Camera *camera, Request *request)
{
	AtomispCameraData *data = cameraData(camera);
	int ret;

	std::map<const Stream *, FrameBuffer *> buffers;
	bool metadataRequired = false;

	for (auto &[stream, buffer] : request->buffers()) {
		/*
		 * If conversion is needed, push the buffer to the converter
		 * queue, it will be handed to the converter in the capture
		 * completion handler.
		 */
		if (data->useConversion_ && stream != data->rawStream_) {
			buffers.emplace(stream, buffer);
			metadataRequired = !!data->swIsp_;
		} else {
			ret = data->video_->queueBuffer(buffer);
			if (ret < 0)
				return ret;
		}
	}

	data->frameInfo_.create(request, metadataRequired);
	if (data->useConversion_) {
		data->conversionQueue_.push({ request, std::move(buffers) });
		if (data->swIsp_)
			data->swIsp_->queueRequest(request->sequence(), request->controls());
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Match and Setup
 */

std::vector<MediaEntity *>
AtomispPipelineHandler::locateSensors(MediaDevice *media)
{
	std::vector<MediaEntity *> entities;

	/*
	 * Gather all the camera sensor entities based on the function they
	 * expose.
	 */
	for (MediaEntity *entity : media->entities()) {
		if (entity->function() == MEDIA_ENT_F_CAM_SENSOR)
			entities.push_back(entity);
	}

	if (entities.empty())
		return {};

	if (atomispQuirks_) {
		/*
		 * Keep the real sensor entities (MEDIA_ENT_F_CAM_SENSOR) as camera
		 * roots for AtomISP split sensor+ISP topologies. Mandatory controls
		 * may only exist on the sensor entity.
		 */
		return entities;
	}

	/*
	 * Sensors can be made of multiple entities. For instance, a raw sensor
	 * can be connected to an ISP, and the combination of both should be
	 * treated as one sensor. To support this, as a crude heuristic, check
	 * the downstream entity from the camera sensor, and if it is an ISP,
	 * use it instead of the sensor.
	 */
	std::vector<MediaEntity *> sensors;

	for (MediaEntity *entity : entities) {
		/*
		 * Locate the downstream entity by following the first link
		 * from a source pad.
		 */
		const MediaLink *link = nullptr;

		for (const MediaPad *pad : entity->pads()) {
			if ((pad->flags() & MEDIA_PAD_FL_SOURCE) &&
			    !pad->links().empty()) {
				link = pad->links()[0];
				break;
			}
		}

		if (!link)
			continue;

		MediaEntity *remote = link->sink()->entity();
		if (remote->function() == MEDIA_ENT_F_PROC_VIDEO_ISP)
			sensors.push_back(remote);
		else
			sensors.push_back(entity);
	}

	/*
	 * Remove duplicates, in case multiple sensors are connected to the
	 * same ISP.
	 */
	std::sort(sensors.begin(), sensors.end());
	auto last = std::unique(sensors.begin(), sensors.end());
	sensors.erase(last, sensors.end());

	return sensors;
}

int AtomispPipelineHandler::resetRoutingTable(V4L2Subdevice *subdev)
{
	/* Reset the media entity routing table to its default state. */
	V4L2Subdevice::Routing routing = {};

	int ret = subdev->getRouting(&routing, V4L2Subdevice::TryFormat);
	if (ret)
		return ret;

	ret = subdev->setRouting(&routing, V4L2Subdevice::ActiveFormat);
	if (ret)
		return ret;

	/*
	 * If the routing table is empty we won't be able to meaningfully use
	 * the subdev.
	 */
	if (routing.empty()) {
		LOG(AtomispPipeline, Error)
			<< "Default routing table of " << subdev->deviceNode()
			<< " is empty";
		return -EINVAL;
	}

	LOG(AtomispPipeline, Debug)
		<< "Routing table of " << subdev->deviceNode()
		<< " reset to " << routing;

	return 0;
}

bool AtomispPipelineHandler::matchDevice(std::shared_ptr<MediaDevice> media,
					const AtomispDriverInfo &info,
					DeviceEnumerator *enumerator)
{
	unsigned int numStreams = 1;

	for (const auto &[name, streams] : info.converters) {
		DeviceMatch converterMatch(name);
		converter_ = acquireMediaDevice(enumerator, converterMatch);
		if (converter_) {
			numStreams = streams;
			break;
		}
	}

	if (info.swIspEnabled) {
		/*
		 * When the software ISP is enabled, the simple pipeline handler
		 * exposes the raw stream, giving a total of two streams. This
		 * is mutually exclusive with the presence of a converter.
		 */
		ASSERT(!converter_);
		numStreams = 2;
	}

	swIspEnabled_ = false; /* AtomISP always outputs YUV; no software debayering */
	atomispQuirks_ = true;
	const GlobalConfiguration &configuration = cameraManager()->_d()->configuration();
	for (const ValueNode &entry :
	     configuration.configuration()["pipelines"]["atomisp"]["supported_devices"]
		     .asList()) {
		auto name = entry["driver"].get<std::string>();
		if (name == info.driver) {
			swIspEnabled_ = entry["software_isp"].get<bool>().value_or(swIspEnabled_);
			LOG(AtomispPipeline, Debug)
				<< "Configuration file overrides software ISP for "
				<< info.driver << " to " << swIspEnabled_;
			break;
		}
	}

	/* Locate the sensors. */
	std::vector<MediaEntity *> sensors = locateSensors(media.get());
	if (sensors.empty()) {
		LOG(AtomispPipeline, Info) << "No sensor found for " << media->deviceNode();
		return false;
	}

	LOG(AtomispPipeline, Debug) << "Sensor found for " << media->deviceNode();

	/*
	 * Create one camera data instance for each sensor and gather all
	 * entities in all pipelines.
	 */
	std::vector<std::unique_ptr<AtomispCameraData>> pipelines;
	std::set<MediaEntity *> entities;

	pipelines.reserve(sensors.size());

	for (MediaEntity *sensor : sensors) {
		std::unique_ptr<AtomispCameraData> data =
			std::make_unique<AtomispCameraData>(this, numStreams, sensor);
		if (!data->isValid()) {
			LOG(AtomispPipeline, Error)
				<< "No valid pipeline for sensor '"
				<< sensor->name() << "', skipping";
			continue;
		}

		for (AtomispCameraData::Entity &entity : data->entities_)
			entities.insert(entity.entity);

		pipelines.push_back(std::move(data));
	}

	if (entities.empty())
		return false;

	/*
	 * Insert all entities in the global entities list. Create and open
	 * V4L2VideoDevice and V4L2Subdevice instances for the corresponding
	 * entities.
	 */
	for (MediaEntity *entity : entities) {
		std::unique_ptr<V4L2VideoDevice> video;
		std::unique_ptr<V4L2Subdevice> subdev;
		int ret;

		switch (entity->type()) {
		case MediaEntity::Type::V4L2VideoDevice:
			video = std::make_unique<V4L2VideoDevice>(entity);
			ret = video->open();
			if (ret < 0) {
				LOG(AtomispPipeline, Error)
					<< "Failed to open " << video->deviceNode()
					<< ": " << strerror(-ret);
				return false;
			}
			break;

		case MediaEntity::Type::V4L2Subdevice:
			subdev = std::make_unique<V4L2Subdevice>(entity);
			ret = subdev->open();
			if (ret < 0) {
				LOG(AtomispPipeline, Error)
					<< "Failed to open " << subdev->deviceNode()
					<< ": " << strerror(-ret);
				return false;
			}

			if (subdev->caps().hasStreams()) {
				/*
				 * Reset the routing table to its default state
				 * to make sure entities are enumerated according
				 * to the default routing configuration.
				 */
				ret = resetRoutingTable(subdev.get());
				if (ret) {
					LOG(AtomispPipeline, Error)
						<< "Failed to reset routes for "
						<< subdev->deviceNode() << ": "
						<< strerror(-ret);
					return false;
				}
			}

			break;

		default:
			break;
		}

		entities_[entity] = { std::move(video), std::move(subdev), {} };
	}

	/* Initialize each pipeline and register a corresponding camera. */
	bool registered = false;

	for (std::unique_ptr<AtomispCameraData> &data : pipelines) {
		int ret = data->init();
		if (ret < 0)
			continue;

		std::set<Stream *> streams;
		std::transform(data->streams_.begin(), data->streams_.end(),
			       std::inserter(streams, streams.end()),
			       [](Stream &stream) { return &stream; });

		const std::string &id = data->sensor_->id();
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), id, streams);
		registerCamera(std::move(camera));
		registered = true;
	}

	return registered;
}

bool AtomispPipelineHandler::match(DeviceEnumerator *enumerator)
{
	std::shared_ptr<MediaDevice> media;

	for (const AtomispDriverInfo &inf : supportedDevices) {
		DeviceMatch dm(inf.driver);
		while ((media = acquireMediaDevice(enumerator, dm))) {
			/*
			 * If match succeeds, return true to let match() be
			 * called again on a new instance of the pipeline
			 * handler. Otherwise keep looping until we do
			 * successfully match one (or run out).
			 */
			if (matchDevice(media, inf, enumerator)) {
				LOG(AtomispPipeline, Debug)
					<< "Matched on device: "
					<< media->deviceNode();
				return true;
			}

			/*
			 * \todo We need to clear the list of media devices
			 * that we've already acquired in the event that we
			 * fail to create a camera. This requires a rework of
			 * DeviceEnumerator, or even how we create pipelines
			 * handlers. This is because at the moment acquired
			 * media devices are only released on pipeline handler
			 * deconstruction, and if we release them any earlier
			 * then DeviceEnumerator::search() will keep returning
			 * the same media devices.
			 */
		}
	}

	return false;
}

V4L2VideoDevice *AtomispPipelineHandler::video(const MediaEntity *entity)
{
	auto iter = entities_.find(entity);
	if (iter == entities_.end())
		return nullptr;

	return iter->second.video.get();
}

V4L2Subdevice *AtomispPipelineHandler::subdev(const MediaEntity *entity)
{
	auto iter = entities_.find(entity);
	if (iter == entities_.end())
		return nullptr;

	return iter->second.subdev.get();
}

/**
 * \brief Acquire all resources needed by the camera pipeline
 * \return nullptr on success, a pointer to the contended pad on error
 */
const MediaPad *AtomispPipelineHandler::acquirePipeline(AtomispCameraData *data)
{
	for (const AtomispCameraData::Entity &entity : data->entities_) {
		const EntityData &edata = entities_[entity.entity];

		if (entity.sink) {
			auto iter = edata.owners.find(entity.sink);
			if (iter != edata.owners.end() && iter->second != data)
				return entity.sink;
		}

		if (entity.source) {
			auto iter = edata.owners.find(entity.source);
			if (iter != edata.owners.end() && iter->second != data)
				return entity.source;
		}
	}

	for (const AtomispCameraData::Entity &entity : data->entities_) {
		EntityData &edata = entities_[entity.entity];

		if (entity.sink)
			edata.owners[entity.sink] = data;
		if (entity.source)
			edata.owners[entity.source] = data;
	}

	return nullptr;
}

void AtomispPipelineHandler::releasePipeline(AtomispCameraData *data)
{
	for (const AtomispCameraData::Entity &entity : data->entities_) {
		EntityData &edata = entities_[entity.entity];

		if (entity.sink) {
			auto iter = edata.owners.find(entity.sink);
			ASSERT(iter->second == data);
			edata.owners.erase(iter);
		}

		if (entity.source) {
			auto iter = edata.owners.find(entity.source);
			ASSERT(iter->second == data);
			edata.owners.erase(iter);
		}
	}
}

REGISTER_PIPELINE_HANDLER(AtomispPipelineHandler, "atomisp")

} /* namespace libcamera */
