/*
Copyright (c) 2010-2025, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "CameraMobileRealSense.h"
#include "util.h"
#include "background_renderer.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>

#include <cstring>

#ifdef RTABMAP_REALSENSE2
#include <rtabmap/core/camera/CameraRealSense2.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <glm/gtx/transform.hpp>
#endif

namespace rtabmap {

bool CameraMobileRealSense::available()
{
#ifdef RTABMAP_REALSENSE2
	return true;
#else
	return false;
#endif
}

CameraMobileRealSense::CameraMobileRealSense(int width, int height, int fps) :
		// No upstream re-localization filter: that filter exists to reject the
		// relocalization jumps of an AR SDK. The odometry here is computed locally
		// and never jumps, but its normal frame-to-frame jitter at 30 Hz does look
		// like a multi-g acceleration, which would make the filter fire constantly
		// and report the tracking as lost.
		CameraMobile(0.0f),
		rsCamera_(0),
		odometry_(0),
		captureThread_(0),
		width_(width),
		height_(height),
		fps_(fps),
		lost_(false),
		lostCounter_(0),
		rateCounter_(0),
		screenWidth_(0),
		screenHeight_(0),
		textureCanvasOffsetX_(0),
		textureCanvasOffsetY_(0)
{
#ifndef RTABMAP_REALSENSE2
	UERROR("RTAB-Map is not built with RealSense2 support!");
#endif
}

CameraMobileRealSense::~CameraMobileRealSense()
{
	close();
}

bool CameraMobileRealSense::init(const std::string & calibrationFolder, const std::string & cameraName)
{
#ifdef RTABMAP_REALSENSE2
	close();

	if(!CameraMobile::init(calibrationFolder, cameraName))
	{
		return false;
	}

	// deviceTColorCamera_ has been set by CameraMobile::init(), use it as local
	// transform so that the poses computed by the odometry below are expressed
	// in rtabmap's world frame (x forward, y left, z up).
	rsCamera_ = new CameraRealSense2("", 0, deviceTColorCamera_);
	rsCamera_->setResolution(width_, height_, fps_);
	rsCamera_->setDepthResolution(width_, height_, fps_);

	if(!rsCamera_->init(calibrationFolder, cameraName))
	{
		UERROR("CameraMobileRealSense: failed to initialize the RealSense camera. Is the "
			   "camera plugged (USB-OTG) and USB permission granted?");
		delete rsCamera_;
		rsCamera_ = 0;
		return false;
	}

	model_ = CameraModel(); // updated on first frame received
	LOGI("CameraMobileRealSense: opened %s", rsCamera_->getSerial().c_str());
	if(rsCamera_->getLocalTransform() != deviceTColorCamera_)
	{
		// See the comment in captureFrame(): the transform is enforced there.
		UWARN("CameraMobileRealSense: CameraRealSense2 did not keep the local transform "
			  "it was given (requested=%s, actual=%s).",
				deviceTColorCamera_.prettyPrint().c_str(),
				rsCamera_->getLocalTransform().prettyPrint().c_str());
	}

	ParametersMap parameters = Parameters::getDefaultParameters();
	// Recover automatically after a short tracking loss instead of staying lost
	// forever. On a reset, odometry resumes from the last computed pose, so the
	// map is not teleported.
	uInsert(parameters, ParametersPair(Parameters::kOdomResetCountdown(), "20"));
	// Don't mask feature extraction with the depth image. Depth aligned on the
	// color image has holes (occlusions, and the stereo module sees nothing on
	// untextured surfaces), and PnP only needs 2d features in the current frame
	// anyway, their 3d counterpart comes from the local map.
	uInsert(parameters, ParametersPair(Parameters::kVisDepthAsMask(), "false"));
	// The local bundle adjustment and the local map size are left at their default
	// (desktop) values: they are the expensive parts on a phone, but they are also
	// what constrains the pose against the local map, and without them the poses
	// are visibly jittery even when the camera is held still. Keeping the streams
	// at 640x480 is what buys the frame rate back instead.
	// The user's mapping settings win over the defaults above.
	uInsert(parameters, odometryParameters_);
	odometry_ = Odometry::create(parameters);
	lost_ = false;

	captureThread_ = new CaptureThread(this);
	captureThread_->start();

	// Don't return until the camera is streaming and the odometry has produced a
	// first pose. SensorCaptureThread treats an empty takeImage() as the end of
	// the stream and kills itself for good (see its "no more data..." branch), so
	// mapping would never start if it began polling before the RealSense is up:
	// CameraMobile::captureImage() only waits 15s, and starting the streams plus
	// initializing the odometry can take longer than that.
	UTimer timer;
	bool streaming = false;
	while(!streaming && timer.elapsed() < 20.0)
	{
		{
			UScopeMutex lock(frameMutex_);
			streaming = frame_.isValid();
		}
		if(!streaming)
		{
			uSleep(100);
		}
	}
	if(!streaming)
	{
		UERROR("CameraMobileRealSense: no frame with a valid pose after %.0f s. The "
			   "camera may not be streaming, or the odometry could not initialize on "
			   "the current view (try pointing at a textured scene).", timer.elapsed());
		close();
		return false;
	}
	LOGI("CameraMobileRealSense: streaming after %.1f s", timer.elapsed());
	return true;
#else
	UERROR("CameraMobileRealSense: RTAB-Map is not built with RealSense2 support!");
	return false;
#endif
}

void CameraMobileRealSense::close()
{
	if(captureThread_)
	{
		captureThread_->join(true);
		delete captureThread_;
		captureThread_ = 0;
	}
#ifdef RTABMAP_REALSENSE2
	// Guarded: CameraRealSense2 and Odometry are only complete types when RTAB-Map
	// is built with RealSense support (they are always null otherwise anyway).
	delete rsCamera_;
	rsCamera_ = 0;
	delete odometry_;
	odometry_ = 0;
#endif
	{
		UScopeMutex lock(frameMutex_);
		frame_ = SensorData();
		framePose_ = Transform();
	}
	lost_ = false;
	lostCounter_ = 0;
	textureCanvas_ = cv::Mat();
	textureCanvasOffsetX_ = 0;
	textureCanvasOffsetY_ = 0;
	CameraMobile::close();
}

std::string CameraMobileRealSense::getSerial() const
{
#ifdef RTABMAP_REALSENSE2
	if(rsCamera_)
	{
		return rsCamera_->getSerial();
	}
#endif
	return "CameraMobileRealSense";
}

// Capture thread
void CameraMobileRealSense::captureFrame()
{
#ifdef RTABMAP_REALSENSE2
	CameraRealSense2 * rsCamera = rsCamera_;
	Odometry * odometry = odometry_;
	if(rsCamera == 0 || odometry == 0)
	{
		uSleep(100);
		return;
	}

	SensorData data = rsCamera->takeImage();
	if(!data.isValid() || data.cameraModels().empty())
	{
		// Camera unplugged or stream interrupted, don't spin on the CPU.
		uSleep(10);
		return;
	}

	// CameraRealSense2 does not necessarily keep the local transform it was given
	// (it rewrites it during init()), and the odometry derives the frame of the
	// poses it returns from it: with the wrong transform the poses come back in the
	// camera optical frame instead of rtabmap's base frame, which rotates the whole
	// map by 90 degrees and makes the AR view look the wrong way. CameraMobile
	// requires device->color-optical here, so enforce it.
	if(data.cameraModels()[0].localTransform() != deviceTColorCamera_)
	{
		CameraModel model = data.cameraModels()[0];
		model.setLocalTransform(deviceTColorCamera_);
		data.setCameraModel(model);
	}

	// Odometry consumes the data and fills its features, which are then reused
	// by the mapping thread.
	OdometryInfo odomInfo;
	Transform pose = odometry->process(data, &odomInfo);

	if(pose.isNull())
	{
		// Losing a single frame is normal (motion blur, a close featureless
		// surface); only tell the user when the tracking is really gone, otherwise
		// the notification fires several times a second.
		++lostCounter_;
		if(!lost_ && lostCounter_ >= 15)
		{
			lost_ = true;
			UWARN("CameraMobileRealSense: odometry lost!");
			this->post(new CameraInfoEvent(0, "OdometryLost", "true"));
		}
		return;
	}
	lostCounter_ = 0;
	if(lost_)
	{
		lost_ = false;
		this->post(new CameraInfoEvent(0, "OdometryLost", "false"));
	}

	// The odometry has to keep up with the camera, otherwise the motion between two
	// processed frames gets too large for the correspondences to be found. Report it
	// when it doesn't, this is the first thing to look at if tracking is lost as soon
	// as the camera moves. Silent while healthy.
	if(++rateCounter_ >= 150)
	{
		const double rate = double(rateCounter_) / rateTimer_.restart();
		rateCounter_ = 0;
		if(rate < 15.0)
		{
			UERROR("CameraMobileRealSense: odometry is only running at %.0f Hz, tracking "
				   "will be lost as soon as the camera moves.", rate);
		}
	}

	UScopeMutex lock(frameMutex_);
	frame_ = data;
	framePose_ = pose;
#else
	uSleep(1000);
#endif
}

// OpenGL thread
SensorData CameraMobileRealSense::updateDataOnRender(Transform & pose)
{
#ifdef RTABMAP_REALSENSE2
	SensorData data;
	{
		UScopeMutex lock(frameMutex_);
		if(!frame_.isValid())
		{
			return SensorData();
		}
		data = frame_;
		pose = framePose_;
		frame_ = SensorData();
	}

	UASSERT(!data.cameraModels().empty());
	const CameraModel & model = data.cameraModels()[0];
	model_ = model;

	// Upload the color image, it is used as background of the AR view. Contrary to
	// the AR SDKs, the image doesn't come from an OES texture, so we have to copy it
	// in a standard 2D texture.
	//
	// The background is drawn on a quad covering the whole viewport (see
	// BackgroundRenderer_kVerticesDevice, which we cannot resize), so the image is
	// letterboxed into a canvas of the viewport aspect ratio to be displayed without
	// distortion. The projection matrix below is built from that canvas, which keeps
	// the 3d overlay aligned with the image.
	if(textureId_ == 0)
	{
		glGenTextures(1, &textureId_);
	}
	if(textureId_ != 0 && !data.imageRaw().empty())
	{
		const cv::Mat & image = data.imageRaw();
		int canvasWidth = image.cols;
		int canvasHeight = image.rows;
		if(screenWidth_ > 0 && screenHeight_ > 0)
		{
			const double screenAspect = double(screenWidth_) / double(screenHeight_);
			const double imageAspect = double(image.cols) / double(image.rows);
			if(screenAspect > imageAspect)
			{
				canvasWidth = int(double(image.rows) * screenAspect + 0.5);
			}
			else if(screenAspect < imageAspect)
			{
				canvasHeight = int(double(image.cols) / screenAspect + 0.5);
			}
		}

		if(textureCanvas_.cols != canvasWidth || textureCanvas_.rows != canvasHeight)
		{
			// Zeroed once, only the image region is overwritten afterwards, so the
			// letterbox borders stay black.
			textureCanvas_ = cv::Mat::zeros(canvasHeight, canvasWidth, CV_8UC4);
			textureCanvasOffsetX_ = (canvasWidth - image.cols) / 2;
			textureCanvasOffsetY_ = (canvasHeight - image.rows) / 2;
		}

		cv::Mat roi = textureCanvas_(cv::Rect(textureCanvasOffsetX_, textureCanvasOffsetY_, image.cols, image.rows));
		cv::cvtColor(image, roi, cv::COLOR_BGR2RGBA);

		glBindTexture(GL_TEXTURE_2D, textureId_);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, textureCanvas_.cols, textureCanvas_.rows, 0, GL_RGBA, GL_UNSIGNED_BYTE, textureCanvas_.data);

		GLint error = glGetError();
		if(error != GL_NO_ERROR)
		{
			LOGE("CameraMobileRealSense: could not allocate texture (0x%x)", error);
			textureId_ = 0;
		}
		else
		{
			// The image is not rotated with the screen, so the mapping between
			// BackgroundRenderer_kVerticesDevice and the texture is constant.
			memcpy(transformed_uvs_, BackgroundRenderer_kVerticesView, 8*sizeof(float));
			uvs_initialized_ = true;
		}
	}

	// OpenGL view and projection matrices matching the letterboxed canvas above:
	// same focal lengths as the color camera, principal point shifted by the
	// letterbox borders.
	const float nearPlane = 0.1f;
	const float farPlane = 100.0f;
	const float canvasWidth = float(textureCanvas_.empty() ? model.imageWidth() : textureCanvas_.cols);
	const float canvasHeight = float(textureCanvas_.empty() ? model.imageHeight() : textureCanvas_.rows);
	const float canvasCx = float(model.cx()) + float(textureCanvasOffsetX_);
	const float canvasCy = float(model.cy()) + float(textureCanvasOffsetY_);
	projectionMatrix_ = glm::mat4(0);
	projectionMatrix_[0][0] = 2.0f * float(model.fx()) / canvasWidth;
	projectionMatrix_[1][1] = 2.0f * float(model.fy()) / canvasHeight;
	projectionMatrix_[2][0] = 1.0f - 2.0f * canvasCx / canvasWidth;
	projectionMatrix_[2][1] = 2.0f * canvasCy / canvasHeight - 1.0f;
	projectionMatrix_[2][2] = -(farPlane + nearPlane) / (farPlane - nearPlane);
	projectionMatrix_[2][3] = -1.0f;
	projectionMatrix_[3][2] = -2.0f * farPlane * nearPlane / (farPlane - nearPlane);

	viewMatrix_ = glm::inverse(glmFromTransform(
			rtabmap::opengl_world_T_rtabmap_world * pose * rtabmap::rtabmap_world_T_opengl_world));

	// Depth is already registered on the color image by CameraRealSense2, so it
	// can be used directly for the occlusion test of the AR view.
	if(!data.depthRaw().empty())
	{
		CameraModel depthModel = model;
		depthModel.setLocalTransform(pose*model.localTransform());
		this->setOcclusionImage(data.depthRaw(), depthModel);
	}

	// Must be done before returning, postUpdate() uses the origin offset set here.
	this->poseReceived(pose, data.stamp());

	return data;
#else
	return SensorData();
#endif
}

} /* namespace rtabmap */
