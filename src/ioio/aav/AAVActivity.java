/*
 * This file is part of the Autonomous Android Vehicle (AAV) application.
 *
 * AAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * AAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with AAV.  If not, see <http://www.gnu.org/licenses/>.
 */

package ioio.aav;

import ioio.lib.api.AnalogInput;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;

import java.util.ArrayList;
import java.util.List;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.WindowManager;

public class AAVActivity extends IOIOActivity implements CvCameraViewListener2 {
	private static final String _TAG = "AAVActivity";

	static final double MIN_CONTOUR_AREA = 100;

	private Mat rgba_image;

	private JavaCameraView opencv_camera_view;
	private ActuatorController main_controller;

	volatile double contour_area = 7;
	volatile Point center_point = new Point(-1, -1);
	Point _screenCenterCoordinates = new Point(-1, -1);
	int _countOutOfFrame = 0;

	Mat hsv_mat;
	Mat processed_mat;
	Mat dilated_mat;
	Scalar lower_threshold;
	Scalar upper_threshold;
	final List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

	SharedPreferences shared_preferences;
	GestureDetector gesture_detector;
	static int tracking_color = 0;

	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
				opencv_camera_view.enableView();
				hsv_mat = new Mat();
				processed_mat = new Mat();
				dilated_mat = new Mat();
			}
				break;
			default: {
				super.onManagerConnected(status);
			}
				break;
			}
		}
	};

	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		// requestWindowFeature(Window.FEATURE_NO_TITLE);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

		setContentView(R.layout.main);

		PreferenceManager.setDefaultValues(this, R.xml.settings, false);
		shared_preferences = PreferenceManager.getDefaultSharedPreferences(this);
		tracking_color = Integer.parseInt(shared_preferences.getString(getString(R.string.color_key), "0"));

		if (tracking_color == 0) {
			lower_threshold = new Scalar(60, 100, 30); // Green
			upper_threshold = new Scalar(130, 255, 255);
		} else if (tracking_color == 1) {
			lower_threshold = new Scalar(160, 50, 90); // Purple
			upper_threshold = new Scalar(255, 255, 255);
		} else if (tracking_color == 2) {
			lower_threshold = new Scalar(1, 50, 150); // Orange
			upper_threshold = new Scalar(60, 255, 255);
		}

		opencv_camera_view = (JavaCameraView) findViewById(R.id.aav_activity_surface_view);
		opencv_camera_view.setCvCameraViewListener(this);

		opencv_camera_view.setMaxFrameSize(176, 144);
		main_controller = new ActuatorController();
		_countOutOfFrame = 0;

		gesture_detector = new GestureDetector(this, new GestureDetector.SimpleOnGestureListener() {
			@Override
			public void onLongPress(MotionEvent e) {
				startActivityForResult(new Intent(getApplicationContext(), SettingsActivity.class), 0);
			}
		});
	}

	@Override
	public boolean onTouchEvent(MotionEvent event) {
		return gesture_detector.onTouchEvent(event);
	};

	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		super.onActivityResult(requestCode, resultCode, data);
		tracking_color = Integer.parseInt(shared_preferences.getString(getString(R.string.color_key), "0"));

		switch (tracking_color) {
		case 0: // Green
			lower_threshold.set(new double[] { 60, 100, 30, 0 });
			upper_threshold.set(new double[] { 130, 255, 255, 0 });
			break;
		case 1: // Purple
			lower_threshold.set(new double[] { 160, 50, 90 });
			upper_threshold.set(new double[] { 255, 255, 255, 0 });
			break;
		case 2: // Orange
			lower_threshold.set(new double[] { 1, 50, 150 });
			upper_threshold.set(new double[] { 60, 255, 255, 0 });
			break;
		default:
			lower_threshold.set(new double[] { 60, 100, 30, 0 });
			upper_threshold.set(new double[] { 130, 255, 255, 0 });
			break;
		}
	}

	@Override
	public void onResume() {
		super.onResume();

		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_7, this, mLoaderCallback);
	}

	@Override
	public void onPause() {
		super.onPause();

		if (opencv_camera_view != null)
			opencv_camera_view.disableView();
	}

	@Override
	public void onDestroy() {
		super.onDestroy();

		if (opencv_camera_view != null)
			opencv_camera_view.disableView();
	}

	@Override
	public void onCameraViewStarted(int width, int height) {
		rgba_image = new Mat(height, width, CvType.CV_8UC4);
		_screenCenterCoordinates.x = rgba_image.size().width / 2;
		_screenCenterCoordinates.y = rgba_image.size().height / 2;
	}

	@Override
	public void onCameraViewStopped() {
		rgba_image.release();
		center_point.x = -1;
		center_point.y = -1;
		main_controller.reset();
	}

	@Override
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		synchronized (inputFrame) {

			rgba_image = inputFrame.rgba();

			double current_contour;

			// In contrast to the C++ interface, Android API captures images in the RGBA format.
			// Also, in HSV space, only the hue determines which color it is. Saturation determines
			// how 'white' the color is, and Value determines how 'dark' the color is.
			Imgproc.cvtColor(rgba_image, hsv_mat, Imgproc.COLOR_RGB2HSV_FULL);

			// int ch[] = {0, 0};
			// MatOfInt fromTo = new MatOfInt(ch);
			// Mat hue = new Mat(rgba_image.size(), CvType.CV_8U);
			// List<Mat> hsvlist = new ArrayList<Mat>();
			// List<Mat> huelist = new ArrayList<Mat>();
			// hsvlist.add(0, hsv_mat);
			// huelist.add(0, hue);
			// Core.mixChannels(hsvlist, huelist, fromTo);
			// hue = huelist.get(0);
			// Mat dst = new Mat();
			// Imgproc.threshold(hue, dst, 0, 255, Imgproc.THRESH_OTSU + Imgproc.THRESH_BINARY);
			// Log.e("dst", String.valueOf(dst.total()));
			//
			// List<Mat> chan = new ArrayList<Mat>();
			// Core.split( hsv_mat, chan);
			// Mat H = hsv_mat.row(0);
			// Mat binary = new Mat();
			// Imgproc.threshold(H, binary, 1 /*ignored for otsu*/, 255, Imgproc.THRESH_OTSU);

			Core.inRange(hsv_mat, lower_threshold, upper_threshold, processed_mat);

			// Imgproc.dilate(processed_mat, dilated_mat, new Mat());
			Imgproc.erode(processed_mat, dilated_mat, new Mat());
			Imgproc.findContours(dilated_mat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
			MatOfPoint2f points = new MatOfPoint2f();
			contour_area = 7;
			for (int i = 0, n = contours.size(); i < n; i++) {
				current_contour = Imgproc.contourArea(contours.get(i));
				if (current_contour > contour_area) {
					contour_area = current_contour;
					// contours.get(i).convertTo(points, CvType.CV_32FC2); // contours.get(x) is a single MatOfPoint, but to use minEnclosingCircle we need to pass a MatOfPoint2f so we need to do a
					// conversion
				}
			}
			if (!points.empty() && contour_area > MIN_CONTOUR_AREA) {
				Imgproc.minEnclosingCircle(points, center_point, null);
				// Core.circle(rgba_image, center_point, 3, new Scalar(255, 0, 0), Core.FILLED);
				Core.circle(rgba_image, center_point, (int) Math.round(Math.sqrt(contour_area / Math.PI)), new Scalar(255, 0, 0), 3, 8, 0);// Core.FILLED);
			}
			contours.clear();

			// double area = 1;
			// Mat circles = new Mat();
			// Imgproc.GaussianBlur(processed_mat, processed_mat, new Size(9, 9), 2, 2);
			// Imgproc.HoughCircles(processed_mat, circles, Imgproc.CV_HOUGH_GRADIENT, 2, processed_mat.rows() / 4, 100, 50, 10, 100);
			//
			// // Draw the circles detected
			// int rows = circles.rows();
			// int elemSize = (int) circles.elemSize(); // Returns 12 (3 * 4bytes in a float)
			// float[] circleData = new float[rows * elemSize / 4];
			//
			// if (circleData.length > 0) {
			// circles.get(0, 0, circleData); // Points to the first element and reads the whole thing into circleData
			// int radius = 0; for (int i = 0; i < circleData.length; i = i + 3) {
			// center_point.x = circleData[i];
			// center_point.y = circleData[i + 1];
			// radius = (int) Math.round(circleData[2]);
			// area = Math.PI * (radius * radius);
			// if (area > contour_area) {
			// contour_area = area;
			// }
			// }
			// if (contour_area > MIN_CONTOUR_AREA)
			// Core.circle(rgba_image, center_point, radius, new Scalar(255, 0, 0), 3);
			// }

		}
		return rgba_image;
	}

	/**
	 * This is the thread on which all the IOIO activity happens. It will be run every time the application is resumed and aborted when it is paused. The method setup() will be called right after a
	 * connection with the IOIO has been established (which might happen several times!). Then, loop() will be called repetitively until the IOIO gets disconnected.
	 */
	class Looper extends BaseIOIOLooper {

		private PwmOutput pwm_pan;
		private PwmOutput pwm_tilt;
		private PwmOutput pwm_motor;
		private PwmOutput pwm_front_wheels;

		private double[] pwm_values = new double[4];

		// IRs
		private AnalogInput front_left_IR, front_right_IR, side_right_IR, side_left_IR;

		boolean is_backing = false;

		int pwm_counter = 0;

		/**
		 * Called every time a connection with IOIO has been established. Typically used to open pins.
		 * 
		 * @throws ConnectionLostException
		 *             When IOIO connection is lost.
		 * @throws InterruptedException
		 * 
		 * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#setup()
		 */
		@Override
		protected void setup() throws ConnectionLostException, InterruptedException {

			try {
				pwm_values = main_controller.getPWMValues();

				pwm_pan = ioio_.openPwmOutput(10, 100); // 9 shield
				pwm_tilt = ioio_.openPwmOutput(6, 100); // 5 shield
				pwm_motor = ioio_.openPwmOutput(27, 100); // screw terminal
				pwm_front_wheels = ioio_.openPwmOutput(12, 100); // 11 shield

				front_left_IR = ioio_.openAnalogInput(40); // A/D 1 shield
				side_left_IR = ioio_.openAnalogInput(41); // A/D 2 shield
				front_right_IR = ioio_.openAnalogInput(43); // A/D 3 shield
				side_right_IR = ioio_.openAnalogInput(42); // A/D 4 shield

				pwm_pan.setPulseWidth((int) pwm_values[0]);
				pwm_tilt.setPulseWidth((int) pwm_values[1]);
				pwm_motor.setPulseWidth((int) pwm_values[2]);
				pwm_front_wheels.setPulseWidth((int) pwm_values[3]);
			} catch (ConnectionLostException e) {
				Log.e(_TAG, e.getMessage());
				throw e;
			}
		}

		/**
		 * Called repetitively while the IOIO is connected.
		 * 
		 * @throws ConnectionLostException
		 *             When IOIO connection is lost.
		 * 
		 * @see ioio.lib.util.AbstractIOIOActivity.IOIOThread#loop()
		 */
		@Override
		public void loop() throws ConnectionLostException {

			try {
				synchronized (main_controller) {

					if (contour_area > MIN_CONTOUR_AREA) {

						main_controller.updatePanTiltPWM(_screenCenterCoordinates, center_point);
						main_controller.ir_sensors.updateIRSensorsVoltage(front_left_IR.getVoltage(), front_right_IR.getVoltage(), side_left_IR.getVoltage(), side_right_IR.getVoltage());

						// if (!main_controller.irSensors.isBacking(front_left_IR.getVoltage(), front_right_IR.getVoltage(), side_left_IR.getVoltage(), side_right_IR.getVoltage(), contour_area)) {

						if (pwm_counter > 8) {
							main_controller.updateMotorPWM(contour_area);
							pwm_counter = 0;
						} else {
							pwm_counter++;

							// if (main_controller.irSensors.checkIRSensors())
							// pwm_counter = 10;
						}
						// }

						// main_controller.irSensors.setIRSensorsVoltage(front_left_IR.getVoltage(), front_right_IR.getVoltage(), side_left_IR.getVoltage(), side_right_IR.getVoltage());

						// if (main_controller.irSensors.isBacking()) {
						// main_controller.calculatePanTiltPWM(_screenCenterCoordinates, center_point);
						// } else {
						// // MUST BE IN THIS ORDER
						// main_controller.calculatePanTiltPWM(_screenCenterCoordinates, center_point);
						// // main_controller.calculateWheelsPWM();
						// main_controller.calculateMotorPWM(contour_area);
						// main_controller.irSensors.checkIRSensors();
						// }

						_countOutOfFrame = 0;
					} else {
						_countOutOfFrame++;
						if (_countOutOfFrame > 20) {
							main_controller.reset();
							_countOutOfFrame = pwm_counter = 0;
						}
					}

					pwm_values = main_controller.getPWMValues();

					pwm_pan.setPulseWidth((int) pwm_values[0]);
					pwm_tilt.setPulseWidth((int) pwm_values[1]);
					pwm_front_wheels.setPulseWidth((int) pwm_values[3]);
					pwm_motor.setPulseWidth((int) pwm_values[2]);
				}
				Thread.sleep(8);

			} catch (InterruptedException e) {
				ioio_.disconnect();
			}
		}

		@Override
		public void disconnected() {
			pwm_pan.close();
			pwm_tilt.close();
			pwm_motor.close();
			pwm_front_wheels.close();
		}
	}

	/**
	 * A method to create our IOIO thread.
	 * 
	 * @see ioio.lib.util.AbstractIOIOActivity#createIOIOThread()
	 */
	@Override
	protected IOIOLooper createIOIOLooper() {
		return new Looper();
	}
}