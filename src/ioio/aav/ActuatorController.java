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

import org.opencv.core.Point;

import android.util.Log;

public class ActuatorController {

	public static final int MIN_PAN_PWM = 600;
	public static final int MAX_PAN_PWM = 2500;
	public static final int MIN_TILT_PWM = 1400;
	public static final int MAX_TILT_PWM = 2250;

	public static final int MID_PAN_PWM = (MAX_PAN_PWM + MIN_PAN_PWM) / 2;
	public static final int MID_TILT_PWM = 1775;//(MAX_TILT_PWM + MIN_TILT_PWM) / 2;
	
	public static final int RANGE_PAN_PWM = MAX_PAN_PWM - MID_PAN_PWM;
	
	public static final int RIGHT_FULL_TURN_WHEELS_PWM = 1200;
	public static final int LEFT_FULL_TURN_WHEELS_PWM = 1750;
	public static final int CENTER_FRONT_WHEELS_PWM = (LEFT_FULL_TURN_WHEELS_PWM + RIGHT_FULL_TURN_WHEELS_PWM) / 2;
	public static final int DIF_FRONT_WHEELS_PWM = LEFT_FULL_TURN_WHEELS_PWM - RIGHT_FULL_TURN_WHEELS_PWM; // difference between the side and the middle of the wheel
	
	public static final int RANGE_WHEELS_PWM = LEFT_FULL_TURN_WHEELS_PWM - CENTER_FRONT_WHEELS_PWM;

	public static final int MOTOR_FORWARD_PWM = 1575; 
	public static final int MOTOR_REVERSE_PWM = 1425;
	public static final int MOTOR_NEUTRAL_PWM = 1500;
	
	public static final int MAX_NEUTRAL_CONTOUR_AREA = 1800;
	public static final int MIN_NEUTRAL_CONTOUR_AREA = 600;

	public double _pwmPan;
	public double _pwmTilt;
	public double _pwmMotor;
	public double _pwmFrontWheels;

	IRSensors irSensors;
	
	double _lastPanPWMValue;
	double _lastMotorPWM;
	int _pulseCounter = 0;
	boolean _wasMoving = false;
	
	Point _lastCenterPoint = new Point(0, 0);

	public ActuatorController() {
		// set the pulse width to be exactly the middle
		_lastPanPWMValue = _pwmPan = MID_PAN_PWM;
		_pwmTilt = MID_TILT_PWM;
		_lastMotorPWM = _pwmMotor = MOTOR_NEUTRAL_PWM;
		_pwmFrontWheels = CENTER_FRONT_WHEELS_PWM;

		irSensors = new IRSensors();
	}	
	
	public synchronized double[] getPWMValues() {
		return new double[] {_pwmPan, _pwmTilt, _pwmMotor, _pwmFrontWheels};
	}
	
	public void updateMotorPWM(double currentContourArea) throws InterruptedException {
		updateWheelsPWM();
		if (currentContourArea > MIN_NEUTRAL_CONTOUR_AREA && currentContourArea < MAX_NEUTRAL_CONTOUR_AREA) {
			// The ESC is intelligent enough to see this as braking.
			if (_lastMotorPWM == MOTOR_FORWARD_PWM && _wasMoving) {
				_pwmMotor = MOTOR_REVERSE_PWM - 200;
			} else if (_lastMotorPWM == MOTOR_REVERSE_PWM && _wasMoving) {
				_pwmMotor = MOTOR_FORWARD_PWM;
			} else {
					_pwmMotor = MOTOR_NEUTRAL_PWM;
			}
			_wasMoving = false;
		} else if (currentContourArea < MIN_NEUTRAL_CONTOUR_AREA) {
			_pwmMotor = MOTOR_FORWARD_PWM;
			_wasMoving = true;
		} else if (currentContourArea > MAX_NEUTRAL_CONTOUR_AREA) {
			if (_lastMotorPWM == MOTOR_NEUTRAL_PWM && !_wasMoving) {
				_pulseCounter = 2;
			}
			_pwmMotor = reverseSequence(_pulseCounter);
			if (_pulseCounter > 0)
				_pulseCounter--;
			_wasMoving = true;			
		}
		_lastMotorPWM = _pwmMotor;
	}
	
	private int reverseSequence(int pulseCounter) {
		return (pulseCounter == 2) ?  MOTOR_REVERSE_PWM - 200 : (pulseCounter == 1) ? MOTOR_NEUTRAL_PWM : MOTOR_REVERSE_PWM;
	}

	private void updateWheelsPWM() {
		if (irSensors.checkIRSensors())
			_pwmFrontWheels = constrain(1.3 * ((MID_PAN_PWM - _pwmPan) / RANGE_PAN_PWM) * RANGE_WHEELS_PWM + CENTER_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
	}

	public double constrain(double input, double min, double max) {
		return (input < min) ? min : (input > max) ? max : input;
	}

	public void reset() {
		_lastPanPWMValue = _pwmPan = MID_PAN_PWM;
		_pwmTilt = MID_TILT_PWM;
		_lastMotorPWM = _pwmMotor = MOTOR_NEUTRAL_PWM;
		_pwmFrontWheels = CENTER_FRONT_WHEELS_PWM;
	}

	
	public class IRSensors {
		boolean _reverseWheels = false;
		boolean _doBacking = false;
		double _frontLeftIRVoltage, _frontRightIRVoltage, _leftSideIRVoltage, _rightSideIRVoltage;
	
		public boolean isBacking(float frontLeftIRVoltage, float frontRightIRVoltage, float leftSideIRVoltage, float rightSideIRVoltage) {
			_frontLeftIRVoltage = frontLeftIRVoltage;
			_frontRightIRVoltage = frontRightIRVoltage;
			_leftSideIRVoltage = leftSideIRVoltage;
			_rightSideIRVoltage = rightSideIRVoltage;
			
			if (_frontLeftIRVoltage > 2.0 || _frontRightIRVoltage > 2.0) {
				_doBacking = true;
				if (!_reverseWheels) {
					_pwmMotor = MOTOR_REVERSE_PWM - 150;
					_reverseWheels = true;
					
					Log.e("MOTOR_NEUTRAL_PWM _doBacking", String.valueOf(_pwmMotor));
				} else {
					_pwmMotor = MOTOR_NEUTRAL_PWM;
					
					Log.e("MOTOR_REVERSE_PWM _doBacking", String.valueOf(_pwmMotor));
				}
			} else {
				_doBacking = false;
				_reverseWheels = false;
			}
			
			
//			if (_frontLeftIRVoltage > 2.0 && _frontRightIRVoltage > 2.0) {
//				_doBacking = true;
//				if (!_reverseWheels) {
//					double diff = _pwmFrontWheels - MID_PAN_PWM; // pos if bigger, neg if less
//					_pwmFrontWheels = MID_PAN_PWM - diff; // reverses the wheels
//					_reverseWheels = true;
//					
//					if (_frontLeftIRVoltage > 1.1)
//						Log.e("<------ _frontLeftIRVoltage", String.valueOf(_frontLeftIRVoltage));
//					else if (_frontRightIRVoltage > 1.1)
//						Log.e("_frontRightIRVoltage ----->", String.valueOf(_frontRightIRVoltage));
//				}
//			}
//			if (_doBacking) {
//				_pwmMotor = MOTOR_REVERSE_PWM; // check
//
//				if (_frontLeftIRVoltage < 1.0 || _frontRightIRVoltage < 1.0) {
//					_doBacking = false;
//					_reverseWheels = false;
//				}
//			}

			return _doBacking;
		}
		
		
		public boolean checkIRSensors() {
			if (_frontLeftIRVoltage > 1.1) {
				_pwmFrontWheels = RIGHT_FULL_TURN_WHEELS_PWM;
//				Log.e("<<<<<<<<< _frontLeftIRVoltage", String.valueOf(_frontLeftIRVoltage));
				return false;
			} else if (_frontRightIRVoltage > 1.1) {
//				_pwmFrontWheels = constrain(_pwmFrontWheels + (_frontRightIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
				_pwmFrontWheels = LEFT_FULL_TURN_WHEELS_PWM;
//				Log.e("_frontRightIRVoltage >>>>>>>>>", String.valueOf(_frontRightIRVoltage));
				return false;
			} else if (_leftSideIRVoltage > 1.5) {
				_pwmFrontWheels = constrain(_pwmFrontWheels - (_leftSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//				Log.e(" < ^^^^^^^^_leftSideIRVoltage", String.valueOf(_leftSideIRVoltage));
				Log.e(" < ^^^^^^^^_pwmFrontWheels", String.valueOf(_pwmFrontWheels));
				return false;
			} else if (_rightSideIRVoltage > 1.5) {
				_pwmFrontWheels = constrain(_pwmFrontWheels + (_rightSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//				Log.e("_rightSideIRVoltage ^^^^^^^ >", String.valueOf(_rightSideIRVoltage));
				Log.e("_pwmFrontWheels ^^^^^^^ >", String.valueOf(_pwmFrontWheels));
				return false;
			}
			return true;
		}

//		public void checkFrontIRSensors() {
//			if (_frontLeftIRVoltage > 1.5) {
//				_pwmFrontWheels = constrain(_pwmFrontWheels - (_frontLeftIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//				Log.e("<****** _frontLeftIRVoltage", String.valueOf(_frontLeftIRVoltage));
//			}
//			if (_frontRightIRVoltage > 1.5) {
//				_pwmFrontWheels = constrain(_pwmFrontWheels + (_frontRightIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//				
//				Log.e("_frontRightIRVoltage *****>", String.valueOf(_frontRightIRVoltage));
//			}
//		}
//
//		public void checkSideIRSensors() {
//			if (_leftSideIRVoltage > 1.5)
//				_pwmFrontWheels = constrain(_pwmFrontWheels - (0.33 * (_leftSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM), RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//			if (_rightSideIRVoltage > 1.5)
//				_pwmFrontWheels = constrain(_pwmFrontWheels + (0.33 * (_rightSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM), RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//		}

		public void updateIRSensorsVoltage(float frontLeftIRVoltage, float frontRightIRVoltage, float leftSideIRVoltage, float rightSideIRVoltage) {
			_frontLeftIRVoltage = frontLeftIRVoltage;
			_frontRightIRVoltage = frontRightIRVoltage;
			_leftSideIRVoltage = leftSideIRVoltage;
			_rightSideIRVoltage = rightSideIRVoltage;
		}
	}

	// ------------------------------------------------------------------------------------------------------------------------------------
	
	Point _increment = new Point(0, 0);
	double _targetTiltPosition = 0.0;

	static final double Kd_X = 0.8;// 003901;//018; // Derivative gain (Kd)

	static final int MID_SCREEN_BOUNDARY = 15;

	public void updatePanTiltPWM(Point screenCenterPoint, Point currentCenterPoint) {
		Point derivativeTerm = new Point(0, 0);

		// --- Set up objects to calculate the error and derivative error
		Point error = new Point(0, 0); // The position error
		Point setpoint = new Point(0, 0);

		setpoint.x = (screenCenterPoint.x - currentCenterPoint.x) * 1.3;
		if ((setpoint.x < -MID_SCREEN_BOUNDARY || setpoint.x > MID_SCREEN_BOUNDARY) && currentCenterPoint.x > 0) {
			if (_lastCenterPoint.x != currentCenterPoint.x) {
				_increment.x = setpoint.x * 0.2;
				_lastPanPWMValue = _pwmPan;
			}
			error.x = (_pwmPan - _increment.x);

			derivativeTerm.x = (_pwmPan - _lastPanPWMValue);

			_lastPanPWMValue = _pwmPan;

			_pwmPan = error.x - constrain(Kd_X * derivativeTerm.x, -9, 9);

			_pwmPan = constrain(_pwmPan, MIN_PAN_PWM, MAX_PAN_PWM);

			_lastCenterPoint.x = currentCenterPoint.x;
		}

		setpoint.y = (currentCenterPoint.y - screenCenterPoint.y) * 0.8;
		if ((setpoint.y < -MID_SCREEN_BOUNDARY || setpoint.y > MID_SCREEN_BOUNDARY) && currentCenterPoint.y > 0) {
			if (_lastCenterPoint.y != currentCenterPoint.y) {
				_targetTiltPosition = (_pwmTilt - setpoint.y);
				_increment.y = setpoint.y * 0.41;
			}
			error.y = (_pwmTilt - _increment.y);

			if (_targetTiltPosition > MID_TILT_PWM && error.y > _targetTiltPosition && error.y > _pwmTilt) {
				_pwmTilt = _targetTiltPosition;
				_increment.y = 0;
			}
			if (_targetTiltPosition > MID_TILT_PWM && error.y < _targetTiltPosition && error.y < _pwmTilt) {
				_pwmTilt = _targetTiltPosition;
				_increment.y = 0;
			} else if (_targetTiltPosition < MID_TILT_PWM && error.y < _targetTiltPosition && error.y < _pwmTilt) {
				_pwmTilt = _targetTiltPosition;
				_increment.y = 0;
			} else if (_targetTiltPosition < MID_TILT_PWM && error.y > _targetTiltPosition && error.y > _pwmTilt) {
				_pwmTilt = _targetTiltPosition;
				_increment.y = 0;
			} else {
				_pwmTilt = error.y;
			}

			_pwmTilt = constrain(_pwmTilt, MIN_TILT_PWM, MAX_TILT_PWM);

			_lastCenterPoint.y = currentCenterPoint.y;
		}
	}
}