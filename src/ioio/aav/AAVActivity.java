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
import android.hardware.SensorManager;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.GestureDetector;
import android.view.MotionEvent;
import android.view.WindowManager;

public class AAVActivity extends IOIOActivity implements CvCameraViewListener2 {
	private static final String _TAG = "AAVActivity";
	
	static final double MIN_CONTOUR_AREA = 100;

	private Mat _rgbaImage;

	private JavaCameraView _openCvCameraView;
	private ActuatorController _mainController;
	
	volatile double _currentContourArea = 7;	
	volatile Point _currentCenterPoint = new Point(-1, -1);
	Point _screenCenterCoordinates = new Point(-1, -1);
	int _countOutOfFrame = 0;

	Mat _hsvMat;
	Mat _processedMat;
	Mat _dilatedMat;
	Scalar _lowerThreshold;
	Scalar _upperThreshold;
	final List<MatOfPoint> _contours = new ArrayList<MatOfPoint>();
		
	public SensorFusion _sensorFusion = null;

	SharedPreferences sharedPreferences;
	GestureDetector gestureDetector;
	static int trackingColor = 0;

	
	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS: {
				_openCvCameraView.enableView();
				_hsvMat = new Mat();
				_processedMat = new Mat();
				_dilatedMat = new Mat();
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
		sharedPreferences = PreferenceManager.getDefaultSharedPreferences(this);
		trackingColor = Integer.parseInt(sharedPreferences.getString(getString(R.string.color_key), "0"));
		
		if (trackingColor == 0) {
			_lowerThreshold = new Scalar(60, 100, 30);  // Green
			_upperThreshold = new Scalar(130, 255, 255);
		} else if (trackingColor == 1) {
			_lowerThreshold = new Scalar(160, 50, 90);  // Purple
			_upperThreshold = new Scalar(255, 255, 255);
		} else if (trackingColor == 2) {
			_lowerThreshold = new Scalar(1, 50, 150);  // Orange
			_upperThreshold = new Scalar(60, 255, 255);	
		}
		
		_openCvCameraView = (JavaCameraView) findViewById(R.id.aav_activity_surface_view);
		_openCvCameraView.setCvCameraViewListener(this);

		_openCvCameraView.setMaxFrameSize(176, 144);
		_mainController = new ActuatorController();
		_countOutOfFrame = 0;
		
		
		// Get a reference to the sensor service
		SensorManager sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		_sensorFusion = new SensorFusion(sensorManager);

		gestureDetector = new GestureDetector(this, new GestureDetector.SimpleOnGestureListener() {
			@Override
		    public void onLongPress(MotionEvent e) {
		    	startActivityForResult(new Intent(getApplicationContext(), SettingsActivity.class), 0);
		    }
		});
	}
	
	@Override
	public boolean onTouchEvent(MotionEvent event) {
	    return gestureDetector.onTouchEvent(event);
	};
	
	@Override
	protected void onActivityResult(int requestCode, int resultCode, Intent data) {
		super.onActivityResult(requestCode, resultCode, data);
		trackingColor = Integer.parseInt(sharedPreferences.getString(getString(R.string.color_key), "0"));
		
		switch (trackingColor) {
		case 0:	// Green
			_lowerThreshold.set(new double[] { 60, 100, 30, 0 });
			_upperThreshold.set(new double[] { 130, 255, 255, 0 });
			break;
		case 1:	// Purple
			_lowerThreshold.set(new double[] { 160, 50, 90 });
			_upperThreshold.set(new double[] { 255, 255, 255, 0 });
			break;
		case 2:	// Orange
			_lowerThreshold.set(new double[] { 1, 50, 150 });
			_upperThreshold.set(new double[] { 60, 255, 255, 0 });
			break;
		default:
			_lowerThreshold.set(new double[] { 60, 100, 30, 0 });
			_upperThreshold.set(new double[] { 130, 255, 255, 0 });
			break;
        }
	}

	@Override
	public void onResume() {
		super.onResume();
		
//		 Restore the sensor listeners when user resumes the application.
		_sensorFusion.initListeners();
		
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_7, this, mLoaderCallback);
	}
	
	@Override
	public void onPause() {
		super.onPause();
		
//		 Unregister sensor listeners to prevent the activity from draining the device's battery.
		_sensorFusion.unregisterListeners();
				
		if (_openCvCameraView != null)
			_openCvCameraView.disableView();
	}

	@Override
	public void onDestroy() {
		super.onDestroy();
		
		// Unregister sensor listeners to prevent the activity from draining the device's battery.
		_sensorFusion.unregisterListeners();
				
		if (_openCvCameraView != null)
			_openCvCameraView.disableView();
	}

	@Override
	public void onCameraViewStarted(int width, int height) {
		_rgbaImage = new Mat(height, width, CvType.CV_8UC4);
		_screenCenterCoordinates.x = _rgbaImage.size().width / 2;
		_screenCenterCoordinates.y = _rgbaImage.size().height / 2;
	}

	@Override
	public void onCameraViewStopped() {
		_rgbaImage.release();
		_currentCenterPoint.x = -1;
		_currentCenterPoint.y = -1;
		_mainController.reset();
	}

	@Override
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		synchronized (inputFrame) {

			_rgbaImage = inputFrame.rgba();
			
			double contourArea;

			// In contrast to the C++ interface, Android API captures images in the RGBA format.
			// Also, in HSV space, only the hue determines which color it is. Saturation determines 
			// how 'white' the color is, and Value determines how 'dark' the color is.
			Imgproc.cvtColor(_rgbaImage, _hsvMat, Imgproc.COLOR_RGB2HSV_FULL);
			
//			int ch[] = {0, 0};
//            MatOfInt fromTo = new MatOfInt(ch);
//			Mat hue = new Mat(_rgbaImage.size(), CvType.CV_8U);
//			List<Mat> hsvlist = new ArrayList<Mat>();
//            List<Mat> huelist = new ArrayList<Mat>();
//            hsvlist.add(0, _hsvMat);
//            huelist.add(0, hue);
//			Core.mixChannels(hsvlist, huelist, fromTo);
//			hue = huelist.get(0);
//			Mat dst = new Mat();
//			Imgproc.threshold(hue, dst, 0, 255, Imgproc.THRESH_OTSU + Imgproc.THRESH_BINARY);
//			Log.e("dst", String.valueOf(dst.total()));
//			
//			List<Mat> chan = new ArrayList<Mat>();
//			Core.split( _hsvMat, chan);
//			Mat H = _hsvMat.row(0);
//			Mat binary = new Mat();
//			Imgproc.threshold(H, binary, 1 /*ignored for otsu*/, 255, Imgproc.THRESH_OTSU);


			Core.inRange(_hsvMat, _lowerThreshold, _upperThreshold, _processedMat);

//			Imgproc.dilate(_processedMat, _dilatedMat, new Mat());
			Imgproc.erode(_processedMat, _dilatedMat, new Mat());
			Imgproc.findContours(_dilatedMat, _contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
			MatOfPoint2f points = new MatOfPoint2f();
			_currentContourArea = 7;
			for (int i = 0, n = _contours.size(); i < n; i++) {
				contourArea = Imgproc.contourArea(_contours.get(i));
				if (contourArea > _currentContourArea) {
					_currentContourArea = contourArea;
					_contours.get(i).convertTo(points, CvType.CV_32FC2); // contours.get(x) is a single MatOfPoint, but to use minEnclosingCircle we need to pass a MatOfPoint2f so we need to do a conversion
				}
			}
			if (!points.empty() && _currentContourArea > MIN_CONTOUR_AREA) {
				Imgproc.minEnclosingCircle(points, _currentCenterPoint, null);
//				Core.circle(_rgbaImage, _currentCenterPoint, 3, new Scalar(255, 0, 0), Core.FILLED);
				 Core.circle(_rgbaImage, _currentCenterPoint, (int) Math.round(Math.sqrt(_currentContourArea / Math.PI)), new Scalar(255, 0, 0), 3, 8, 0);//Core.FILLED);
			}
			_contours.clear();
			
			
			
//			double area = 1;
//			Mat circles = new Mat();
//			Imgproc.GaussianBlur(_processedMat, _processedMat, new Size(9, 9), 2, 2); 
//			Imgproc.HoughCircles(_processedMat, circles, Imgproc.CV_HOUGH_GRADIENT, 2, _processedMat.rows() / 4, 100, 50, 10, 100);
//			
//			//  Draw the circles detected 
//			int rows = circles.rows(); 
//			int elemSize = (int) circles.elemSize(); // Returns 12 (3 * 4bytes in a float) 
//			float[] circleData = new float[rows * elemSize / 4];
//			
//			if (circleData.length > 0) { 
//				circles.get(0, 0, circleData); // Points to the first element and reads the whole thing into circleData 
//				int radius = 0; for (int i = 0; i < circleData.length; i = i + 3) { 
//					_currentCenterPoint.x = circleData[i]; 
//					_currentCenterPoint.y = circleData[i + 1]; 
//					radius = (int) Math.round(circleData[2]); 
//					area = Math.PI * (radius * radius); 
//					if (area > _currentContourArea) { 
//						_currentContourArea = area; 
//					} 
//				} 
//				if (_currentContourArea > MIN_CONTOUR_AREA) 
//					Core.circle(_rgbaImage, _currentCenterPoint, radius, new Scalar(255, 0, 0), 3); 
//			}

			

		}
		return _rgbaImage;
	}

	/**
	 * This is the thread on which all the IOIO activity happens. It will be run every time the application is resumed and aborted when it is paused. The method setup() will be called right after a
	 * connection with the IOIO has been established (which might happen several times!). Then, loop() will be called repetitively until the IOIO gets disconnected.
	 */
	class Looper extends BaseIOIOLooper {

		private PwmOutput _pwmPan;
		private PwmOutput _pwmTilt;
		private PwmOutput _pwmMotor;
		private PwmOutput _pwmFrontWheels;

		private double[] _pwmValues = new double[4];

		// IRs
		private AnalogInput _frontLeftIR, _frontRightIR, _rightSideIR, _leftSideIR;
		
		boolean isBacking = false;
		
		int _pwmThresholdCounter = 0;

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
				_pwmValues = _mainController.getPWMValues();

				_pwmPan = ioio_.openPwmOutput(10, 100); // 9 shield
				_pwmTilt = ioio_.openPwmOutput(6, 100); // 5 shield
				_pwmMotor = ioio_.openPwmOutput(27, 100); // screw terminal
				_pwmFrontWheels = ioio_.openPwmOutput(12, 100); // 11 shield
				
				_frontLeftIR = ioio_.openAnalogInput(40);  // A/D 1 shield
				_leftSideIR = ioio_.openAnalogInput(41);  // A/D 2 shield
				_frontRightIR = ioio_.openAnalogInput(43); // A/D 3 shield
				_rightSideIR = ioio_.openAnalogInput(42); // A/D 4 shield
				
				_pwmPan.setPulseWidth((int) _pwmValues[0]);
				_pwmTilt.setPulseWidth((int) _pwmValues[1]);
				_pwmMotor.setPulseWidth((int) _pwmValues[2]);
				_pwmFrontWheels.setPulseWidth((int) _pwmValues[3]);
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
				synchronized (_mainController) {
										
				if (_currentContourArea > MIN_CONTOUR_AREA) {
					
					_mainController.updatePanTiltPWM(_screenCenterCoordinates, _currentCenterPoint);
					_mainController.irSensors.updateIRSensorsVoltage(_frontLeftIR.getVoltage(), _frontRightIR.getVoltage(), _leftSideIR.getVoltage(), _rightSideIR.getVoltage());
					
//					if (!_mainController.irSensors.isBacking(_frontLeftIR.getVoltage(), _frontRightIR.getVoltage(), _leftSideIR.getVoltage(), _rightSideIR.getVoltage(), _currentContourArea)) {
						
					if (_pwmThresholdCounter > 8) {
						_mainController.updateMotorPWM(_currentContourArea);
						_pwmThresholdCounter = 0;
					} else {
						_pwmThresholdCounter++;

//						if (_mainController.irSensors.checkIRSensors())
//							_pwmThresholdCounter = 10;
				    }
//					}
										
//					_mainController.irSensors.setIRSensorsVoltage(_frontLeftIR.getVoltage(), _frontRightIR.getVoltage(), _leftSideIR.getVoltage(), _rightSideIR.getVoltage());
					
//					if (_mainController.irSensors.isBacking()) {
//						_mainController.calculatePanTiltPWM(_screenCenterCoordinates, _currentCenterPoint);
//					} else {
//						// MUST BE IN THIS ORDER
//						_mainController.calculatePanTiltPWM(_screenCenterCoordinates, _currentCenterPoint);
////						_mainController.calculateWheelsPWM();
//						_mainController.calculateMotorPWM(_currentContourArea);
//						_mainController.irSensors.checkIRSensors();
//					}
					
					_countOutOfFrame = 0;
				} else {
					_countOutOfFrame++;
					if (_countOutOfFrame > 20) {
						_mainController.reset();
						_countOutOfFrame = _pwmThresholdCounter = 0;
					}
				}
				
				_pwmValues = _mainController.getPWMValues();

				_pwmPan.setPulseWidth((int) _pwmValues[0]);
				_pwmTilt.setPulseWidth((int) _pwmValues[1]);
				_pwmFrontWheels.setPulseWidth((int) _pwmValues[3]);				
				_pwmMotor.setPulseWidth((int) _pwmValues[2]);
				}
				Thread.sleep(8);

			} catch (InterruptedException e) {
				ioio_.disconnect();
			}
		}

		@Override
		public void disconnected() {
			_pwmPan.close();
			_pwmTilt.close();
			_pwmMotor.close();
			_pwmFrontWheels.close();
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