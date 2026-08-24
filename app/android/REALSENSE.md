# RealSense support in RTAB-Map for Android

The Android app can map with an Intel RealSense D400 camera (D435, D435i, D455, ...)
connected to the phone with a USB-OTG cable, using the same `CameraRealSense2`
driver as the desktop version.

Selected in the app with **Settings → Mapping → Camera Driver → `RealSense (USB)`**
(driver id `4`, next to the existing Tango / ARCore / AREngine drivers).

## How it works

| Piece | Role |
|---|---|
| `src/com/intel/realsense/librealsense/DeviceWatcher.java` | Requests the USB permission, opens the device with `UsbManager` and hands the file descriptor to librealsense through its two JNI entry points. |
| `jni/CameraMobileRealSense.{h,cpp}` | `CameraMobile` subclass wrapping `rtabmap::CameraRealSense2`. Grabs frames and computes visual odometry on a background thread, uploads the color texture on the OpenGL thread. |
| `jni/RTABMapApp.cpp` | Instantiates the driver for `cameraDriver == 4`. |

Two things differ from the AR SDK drivers:

* **Poses.** A D400 has no pose stream, so `CameraMobileRealSense` runs RTAB-Map's
  visual odometry on the RGB-D stream itself and feeds the result to `CameraMobile`
  exactly like an AR SDK would. Everything downstream (rendering, mapping,
  measuring, export) is unchanged. The odometry uses the mapping parameters set in
  the app's settings, plus `Odom/ResetCountdown=20` so a short tracking loss
  recovers by itself.
* **Orientation.** The camera is not attached to the device, so the image is never
  rotated with the screen and `setScreenRotationAndSize()` is a no-op. The AR
  background is drawn from the camera's own point of view, with a projection matrix
  built from its intrinsics, so it is stretched to the viewport aspect.

The USB permission is a `UsbManager` permission, not an Android runtime permission:
it is requested by `DeviceWatcher` when a scan starts (the app also declares a
`USB_DEVICE_ATTACHED` intent-filter, so plugging a camera offers to open RTAB-Map
and grants the permission persistently). The `CAMERA` runtime permission is still
requested first, as for the other drivers.

## Building

### With docker (tested route)

`docker/noble/android/Dockerfile` (the `introlab3it/rtabmap:android-noble-deps`
image) builds librealsense along with the other dependencies, and
`docker/noble/android/rtabmap_apiXX` builds the apk. To avoid rebuilding the whole
deps image, apply just the librealsense layer on top of the published one and point
`BASE_IMAGE` at the result:

```bash
docker pull introlab3it/rtabmap:android-noble-deps

# a Dockerfile with the librealsense RUN block of docker/noble/android/Dockerfile,
# FROM the image above
docker build -f Dockerfile.librealsense -t rtabmap-android-noble-deps-realsense .

docker build -f docker/noble/android/rtabmap_apiXX/Dockerfile \
    --build-arg BASE_IMAGE=rtabmap-android-noble-deps-realsense \
    --build-arg API_VERSION=30 \
    -t rtabmap:android30-realsense .

CID=$(docker create rtabmap:android30-realsense)
docker cp $CID:/root/rtabmap-tango/build/arm64-v8a/app/android/bin/RTABMap-debug.apk .
docker rm $CID

adb install -r RTABMap-debug.apk
```

`ant release` leaves the release apk unsigned, so `RTABMap-debug.apk` is the one that
installs directly. Note that ant (from the deprecated SDK tools r25) only writes a
v1/JAR signature, which Android 11+ rejects for a targetSdk >= 30 apk
(`INSTALL_PARSE_FAILED_NO_CERTIFICATES`); `rtabmap.bash` therefore re-signs it with
`apksigner` after ant. To sign the release apk with your own key:

```bash
BT=$ANDROID_HOME/build-tools/34.0.0
$BT/zipalign -p -f 4 RTABMap-release-unsigned.apk RTABMap.apk
$BT/apksigner sign --ks <your.keystore> RTABMap.apk
$BT/apksigner verify --verbose RTABMap.apk
```

### On the host

librealsense2 has to be cross-compiled for Android first:

```bash
export ANDROID_NDK=$HOME/Android/Sdk/ndk/25.1.8937393
./scripts/install_librealsense_android.bash <librealsense_source_dir> /opt/android arm64-v8a 24
```

The options that matter (see the script for the full list):

* `FORCE_RSUSB_BACKEND=ON` — mandatory on Android: the SDK cannot access
  `/dev/bus/usb`, hence the file descriptor handed over from Java.
* `BUILD_ROSBAG2=OFF` — otherwise its `fastcdr` dependency is left dangling in
  `realsense2Targets.cmake` and breaks `find_package(realsense2)` downstream.
* `BUILD_RS2_ALL=OFF` — the bundled static library is not installed anyway.
* `CMAKE_CXX_FLAGS=-include cerrno` — the vendored nlohmann/json uses `errno`
  without including it, which NDK r21's libc++ does not pull in transitively.

Then configure RTAB-Map for Android as usual. `find_package(realsense2)` picks it up
through `CMAKE_FIND_ROOT_PATH` and the summary should show:

```
-- Android build info:
...
--   With RealSense = YES
```

## Things that had to be handled

Found while bringing this up on a D435 + Galaxy Z Flip (all verified on device):

* **The local transform is not preserved.** `CameraRealSense2` hands back a model
  whose `localTransform()` is the *inverse* of the one it was constructed with
  (requested `rpy=-1.5708,0,-1.5708`, actual `rpy=0,-1.5708,0`). rtabmap's odometry
  derives the frame of its poses from that transform, so the poses came back in the
  camera optical frame instead of rtabmap's base frame: the map was rotated 90
  degrees (a room extruding upwards instead of lying on the ground) and the AR view
  pointed the wrong way. `captureFrame()` enforces the expected transform on the
  data before the odometry sees it. The two places `CameraRealSense2` rewrites its
  local transform are both guarded by `dualMode_`/`odometryProvided_`, neither of
  which applies to a plain D435, so the mechanism is not understood yet and this is
  worth reporting upstream rather than relying on.
* **Don't reuse the AR re-localization filter.** `CameraMobile`'s
  `upstreamRelocalizationAccThr` rejects the pose jumps of an AR SDK. Locally
  computed odometry has no jumps, but its normal jitter at 30 Hz looks like a
  multi-g acceleration, so the filter fired constantly, flagged the tracking as bad
  and made every frame report a 9999 covariance (i.e. lost odometry).
* **The odometry has to stay real-time.** At 5-12 Hz the motion between two
  processed frames is too large for the correspondences to be found and tracking is
  lost as soon as the camera moves. 848x480 (the desktop default) roughly halves the
  rate, because librealsense uses its generic non-SIMD `align` implementation on
  Android. 640x480 with the local bundle adjustment disabled and a smaller local map
  gives `capture=17ms odom=16ms`, i.e. a full 30 Hz.
* **A frame must not be published before the camera is streaming.**
  `SensorCaptureThread` treats one empty `takeImage()` as the end of the stream and
  stops for good, and `CameraMobile::captureImage()` only waits 15 s, which is less
  than the RealSense takes to come up. `init()` therefore waits for a first frame
  with a valid pose. For the same reason, `CameraMobile::updateOnRender()` no longer
  overwrites data that hasn't been consumed with an empty frame: a 30 Hz camera on a
  60 Hz display has nothing new to return on half the renders, and clobbering the
  published frame killed the capture thread within seconds.

## Notes and limitations

* Streams are configured at 640x480@30 for both color and depth, which every D400
  model supports and keeps the visual odometry real-time on a phone.
* The device needs USB host (OTG) support. A D435 draws up to ~700 mA, so a phone
  that cannot supply it needs a powered OTG hub.
* Only `arm64-v8a` is covered by the build recipes, like the rest of the Android app.
* This driver is not part of the `Auto` camera driver selection: it has to be picked
  explicitly, since a USB camera is never the expected default.
* A D435 has no IMU, so nothing gravity-aligns the map: its orientation is the one
  the camera had on the first frame. Start with the camera roughly level. A D435i
  would allow feeding `Odom/GravitySigma` instead.
* If the odometry stays lost for more than 15 s, `CameraMobile::captureImage()`
  times out and `SensorCaptureThread` stops for good, which ends the session. This
  affects every driver, it is just easier to reach with locally computed odometry.
