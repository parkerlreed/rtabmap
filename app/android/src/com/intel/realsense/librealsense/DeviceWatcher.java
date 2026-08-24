/*
 * Copyright 2019 Intel Corporation. All Rights Reserved.
 * Copyright 2025 IntRoLab - Universite de Sherbrooke
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// !!! The package and class names below cannot be changed !!!
//
// librealsense2 is built with its Android (usbhost) backend, which cannot
// enumerate USB devices by itself: an Android app has no direct access to
// /dev/bus/usb, only the java UsbManager can open a device. librealsense
// therefore exposes two JNI entry points, compiled in librealsense2 as
//    Java_com_intel_realsense_librealsense_DeviceWatcher_nAddUsbDevice
//    Java_com_intel_realsense_librealsense_DeviceWatcher_nRemoveUsbDevice
// (see librealsense/src/usbhost/enumerator-usbhost.cpp), to which a USB file
// descriptor opened on the java side is handed over. As JNI resolves native
// methods from the fully qualified java name, this class must stay in the
// com.intel.realsense.librealsense package and keep this exact name.
//
// This is a slimmed down version of librealsense's own DeviceWatcher /
// Enumerator / UsbUtilities classes (they depend on androidx, which this app
// doesn't use) and the native methods are linked in libNativeRTABMap.so
// instead of a separate librealsense2.so.
package com.intel.realsense.librealsense;

import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbManager;
import android.util.Log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Opens Intel RealSense USB devices and hands their file descriptor to librealsense. */
public class DeviceWatcher {

	private static final String TAG = "RTABMap DeviceWatcher";

	/** Intel's USB vendor id, all RealSense cameras use it. */
	public static final int INTEL_VENDOR_ID = 0x8086;

	private static final String ACTION_USB_PERMISSION = "com.introlab.rtabmap.USB_PERMISSION";

	static {
		// The native methods below live in libNativeRTABMap.so, which is loaded by
		// RTABMapLib's static initializer (it has to preload libtango_client_api.so
		// first, so we don't call System.loadLibrary() ourselves here).
		try {
			Class.forName("com.introlab.rtabmap.RTABMapLib");
		}
		catch(ClassNotFoundException e) {
			Log.e(TAG, "Could not load RTABMapLib: " + e.getMessage());
		}
	}

	/** Notified when a RealSense camera becomes usable (or stops being usable). */
	public interface Listener {
		void onDeviceAttached();
		void onDeviceDetached();
	}

	private static DeviceWatcher mInstance = null;

	/**
	 * Starts watching for RealSense cameras. Idempotent, the listener of the
	 * last call is the one used. release() should be called when done.
	 */
	public static synchronized void init(Context context, Listener listener) {
		if(mInstance == null) {
			mInstance = new DeviceWatcher(context.getApplicationContext());
		}
		mInstance.mListener = listener;
		mInstance.refresh();
	}

	public static synchronized void release() {
		if(mInstance != null) {
			mInstance.close();
			mInstance = null;
		}
	}

	/** Number of RealSense cameras currently opened and handed to librealsense. */
	public static synchronized int getDeviceCount() {
		return mInstance == null ? 0 : mInstance.openedDeviceCount();
	}

	/** True if at least one Intel USB device is plugged, whatever its permission state. */
	public static boolean isDeviceConnected(Context context) {
		return !listRealSenseDevices(context).isEmpty();
	}

	private static List<UsbDevice> listRealSenseDevices(Context context) {
		List<UsbDevice> devices = new ArrayList<UsbDevice>();
		UsbManager usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
		if(usbManager == null) {
			return devices;
		}
		HashMap<String, UsbDevice> devicesMap = usbManager.getDeviceList();
		for(Map.Entry<String, UsbDevice> entry : devicesMap.entrySet()) {
			if(entry.getValue().getVendorId() == INTEL_VENDOR_ID) {
				devices.add(entry.getValue());
			}
		}
		return devices;
	}

	private final Context mContext;
	private final Map<String, Descriptor> mDescriptors = new LinkedHashMap<String, Descriptor>();
	private Listener mListener = null;
	private boolean mClosed = false;

	private static class Descriptor {
		Descriptor(String name, int fileDescriptor, UsbDeviceConnection connection) {
			this.name = name;
			this.fileDescriptor = fileDescriptor;
			this.connection = connection;
		}
		final String name;
		final int fileDescriptor;
		final UsbDeviceConnection connection;
	}

	private final BroadcastReceiver mReceiver = new BroadcastReceiver() {
		@Override
		public void onReceive(Context context, Intent intent) {
			// ACTION_USB_DEVICE_ATTACHED, ACTION_USB_DEVICE_DETACHED or the answer
			// of a permission request: refresh() figures out what changed.
			Log.i(TAG, "onReceive: " + intent.getAction());
			refresh();
		}
	};

	private DeviceWatcher(Context context) {
		mContext = context;
		// The app doesn't declare a targetSdkVersion (so it is the minSdkVersion,
		// see AndroidManifest.xml.in), the RECEIVER_NOT_EXPORTED flag introduced
		// in API 33 is therefore not required here.
		mContext.registerReceiver(mReceiver, new IntentFilter(ACTION_USB_PERMISSION));
		mContext.registerReceiver(mReceiver, new IntentFilter(UsbManager.ACTION_USB_DEVICE_ATTACHED));
		mContext.registerReceiver(mReceiver, new IntentFilter(UsbManager.ACTION_USB_DEVICE_DETACHED));
	}

	/**
	 * Synchronizes librealsense with the USB devices currently plugged: removes
	 * the ones that are gone, asks permission for the new ones and opens those
	 * we are allowed to open.
	 */
	private synchronized void refresh() {
		if(mClosed) {
			return;
		}

		UsbManager usbManager = (UsbManager) mContext.getSystemService(Context.USB_SERVICE);
		if(usbManager == null) {
			Log.e(TAG, "USB service not available on this device");
			return;
		}

		Map<String, UsbDevice> connected = new LinkedHashMap<String, UsbDevice>();
		for(UsbDevice device : listRealSenseDevices(mContext)) {
			connected.put(device.getDeviceName(), device);
		}

		boolean detached = false;
		Iterator<Map.Entry<String, Descriptor>> iter = mDescriptors.entrySet().iterator();
		while(iter.hasNext()) {
			Map.Entry<String, Descriptor> entry = iter.next();
			if(!connected.containsKey(entry.getKey())) {
				removeDevice(entry.getValue());
				iter.remove();
				detached = true;
			}
		}

		boolean attached = false;
		for(Map.Entry<String, UsbDevice> entry : connected.entrySet()) {
			if(mDescriptors.containsKey(entry.getKey())) {
				continue;
			}
			UsbDevice device = entry.getValue();
			if(!usbManager.hasPermission(device)) {
				Log.i(TAG, "Requesting USB permission for " + device.getDeviceName());
				// The broadcast sent back on the user's answer triggers refresh() again.
				usbManager.requestPermission(device, PendingIntent.getBroadcast(
						mContext, 0, new Intent(ACTION_USB_PERMISSION), PendingIntent.FLAG_IMMUTABLE));
				continue;
			}
			if(addDevice(usbManager, device)) {
				attached = true;
			}
		}

		if(mListener != null) {
			if(attached) {
				mListener.onDeviceAttached();
			}
			if(detached) {
				mListener.onDeviceDetached();
			}
		}
	}

	private boolean addDevice(UsbManager usbManager, UsbDevice device) {
		UsbDeviceConnection connection = usbManager.openDevice(device);
		if(connection == null) {
			Log.e(TAG, "Could not open USB device " + device.getDeviceName());
			return false;
		}
		Descriptor desc = new Descriptor(device.getDeviceName(), connection.getFileDescriptor(), connection);
		// The connection (and thus the file descriptor) is kept opened as long as
		// librealsense uses the device, it is closed in removeDevice() below.
		mDescriptors.put(desc.name, desc);
		Log.i(TAG, String.format("Adding USB device %s (vid=0x%04x pid=0x%04x fd=%d)",
				desc.name, device.getVendorId(), device.getProductId(), desc.fileDescriptor));
		nAddUsbDevice(desc.name, desc.fileDescriptor);
		return true;
	}

	private void removeDevice(Descriptor desc) {
		Log.i(TAG, "Removing USB device " + desc.name);
		nRemoveUsbDevice(desc.fileDescriptor);
		desc.connection.close();
	}

	private synchronized int openedDeviceCount() {
		return mDescriptors.size();
	}

	private synchronized void close() {
		if(mClosed) {
			return;
		}
		mClosed = true;
		try {
			mContext.unregisterReceiver(mReceiver);
		}
		catch(IllegalArgumentException e) {
			Log.w(TAG, "Receiver was already unregistered: " + e.getMessage());
		}
		Iterator<Map.Entry<String, Descriptor>> iter = mDescriptors.entrySet().iterator();
		while(iter.hasNext()) {
			removeDevice(iter.next().getValue());
			iter.remove();
		}
		mListener = null;
	}

	private static native void nAddUsbDevice(String deviceName, int fileDescriptor);
	private static native void nRemoveUsbDevice(int fileDescriptor);
}
