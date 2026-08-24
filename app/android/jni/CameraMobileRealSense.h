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

#ifndef CAMERAMOBILEREALSENSE_H_
#define CAMERAMOBILEREALSENSE_H_

#include "CameraMobile.h"

#include <rtabmap/core/Version.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/utilite/UMutex.h>
#include <rtabmap/utilite/UThread.h>
#include <rtabmap/utilite/UTimer.h>

namespace rtabmap {

class CameraRealSense2;
class Odometry;

/**
 * RealSense (D400 series) camera plugged over USB-OTG.
 *
 * Contrary to the AR SDK based drivers (Tango, ARCore, AREngine), a RealSense
 * camera doesn't provide any pose, so this class computes visual odometry on
 * the RGB-D stream and feeds the resulting pose to CameraMobile the same way
 * an AR SDK would. Everything downstream (rendering, mapping, measuring)
 * therefore behaves exactly like with the other drivers.
 *
 * Frames are grabbed and odometry is computed on a background thread, the
 * OpenGL thread only picks the latest processed frame in updateDataOnRender()
 * to upload the color texture.
 *
 * The USB device itself is opened on the Java side (see
 * app/android/src/com/intel/realsense/librealsense/DeviceWatcher.java), which
 * hands the USB file descriptor to librealsense's Android backend before this
 * camera is initialized.
 */
class CameraMobileRealSense : public CameraMobile {
public:
	static bool available();

public:
	CameraMobileRealSense(
			int width = 640,
			int height = 480,
			int fps = 30);
	virtual ~CameraMobileRealSense();

	virtual bool init(const std::string & calibrationFolder = ".", const std::string & cameraName = "");
	virtual void close();
	virtual std::string getSerial() const;

	// The rotation is ignored: the camera is not attached to the device, so its
	// images should never be rotated with the screen. The viewport size is kept to
	// letterbox the image in updateDataOnRender().
	virtual void setScreenRotationAndSize(ScreenRotation colorCameraToDisplayRotation, int width, int height)
	{
		screenWidth_ = width;
		screenHeight_ = height;
	}

	// Parameters used to create the odometry, should be called before init().
	void setOdometryParameters(const ParametersMap & parameters) {odometryParameters_ = parameters;}

protected:
	// Called from the OpenGL thread.
	virtual SensorData updateDataOnRender(Transform & pose);

private:
	// Grabs a frame and computes its odometry, called from the capture thread.
	void captureFrame();

	class CaptureThread : public UThread
	{
	public:
		CaptureThread(CameraMobileRealSense * camera) : camera_(camera) {}
	protected:
		virtual void mainLoop() {camera_->captureFrame();}
	private:
		CameraMobileRealSense * camera_;
	};
	friend class CaptureThread;

	CameraRealSense2 * rsCamera_;
	Odometry * odometry_;
	CaptureThread * captureThread_;
	ParametersMap odometryParameters_;

	UMutex frameMutex_;
	SensorData frame_;
	Transform framePose_;

	int width_;
	int height_;
	int fps_;
	bool lost_;
	unsigned int lostCounter_;
	unsigned int rateCounter_;
	UTimer rateTimer_;

	int screenWidth_;
	int screenHeight_;
	// Color image padded to the viewport aspect ratio, uploaded as background texture.
	cv::Mat textureCanvas_;
	int textureCanvasOffsetX_;
	int textureCanvasOffsetY_;
};

} /* namespace rtabmap */
#endif /* CAMERAMOBILEREALSENSE_H_ */
