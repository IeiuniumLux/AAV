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

public class ActuatorController {
	private static final String _TAG = "ActuatorController";

	public static final int MIN_PAN_PWM = 600;
	public static final int MAX_PAN_PWM = 2500;
	public static final int MIN_TILT_PWM = 1400;
	public static final int MAX_TILT_PWM = 2250;

	public static final int MID_PAN_PWM = (MAX_PAN_PWM + MIN_PAN_PWM) / 2;
	public static final int MID_TILT_PWM = 1800;// (MAX_TILT_PWM + MIN_TILT_PWM) / 2;

	public static final int RANGE_PAN_PWM = MAX_PAN_PWM - MID_PAN_PWM;

	public static final int RIGHT_FULL_TURN_WHEELS_PWM = 1200;
	public static final int LEFT_FULL_TURN_WHEELS_PWM = 1800;
	public static final int CENTER_FRONT_WHEELS_PWM = (LEFT_FULL_TURN_WHEELS_PWM + RIGHT_FULL_TURN_WHEELS_PWM) / 2;

	public static final int RANGE_WHEELS_PWM = LEFT_FULL_TURN_WHEELS_PWM - CENTER_FRONT_WHEELS_PWM;

	public static final int MOTOR_FORWARD_PWM = 1578;
	public static final int MOTOR_REVERSE_PWM = 1420;
	public static final int MOTOR_NEUTRAL_PWM = 1500;

	public static final int MAX_NEUTRAL_CONTOUR_AREA = 1700;
	public static final int MIN_NEUTRAL_CONTOUR_AREA = 800;

	public double _pwmPan;
	public double _pwmTilt;
	public double _pwmMotor;
	public double _pwmFrontWheels;

	IRSensors _irSensors;

	double _lastPanPWM;
	double _lastMotorPWM;
	int _pulseCounter = 0;
	boolean _wasMoving = false;

	Point _lastCenterPoint = new Point(0, 0);

	public ActuatorController() {
		// set the pulse width to be exactly the middle
		_lastPanPWM = _pwmPan = MID_PAN_PWM;
		_pwmTilt = MID_TILT_PWM;
		_lastMotorPWM = _pwmMotor = MOTOR_NEUTRAL_PWM;
		_pwmFrontWheels = CENTER_FRONT_WHEELS_PWM;

		_irSensors = new IRSensors();
	}

	public synchronized double[] getPWMValues() {
		return new double[] { _pwmPan, _pwmTilt, _pwmMotor, _pwmFrontWheels };
	}

	public void updateMotorPWM(double currentContourArea) throws InterruptedException {
		updateWheelsPWM();
		if (currentContourArea > MIN_NEUTRAL_CONTOUR_AREA && currentContourArea < MAX_NEUTRAL_CONTOUR_AREA) {
			_pwmMotor = (_wasMoving) ? MOTOR_REVERSE_PWM - 250 : MOTOR_NEUTRAL_PWM;
			_wasMoving = false;
			_pulseCounter = 2;
		} else if (currentContourArea < MIN_NEUTRAL_CONTOUR_AREA) {
			_pwmMotor = MOTOR_FORWARD_PWM;
			_wasMoving = true;
			_pulseCounter = 2;
		} else if (currentContourArea > MAX_NEUTRAL_CONTOUR_AREA) {
			_pwmMotor = reverseSequence(_pulseCounter);
			if (_pulseCounter > 0)
				_pulseCounter--;
			_wasMoving = false;
		}
		_lastMotorPWM = _pwmMotor;
	}

	private int reverseSequence(int pulseCounter) {
		return (pulseCounter == 2) ? MOTOR_REVERSE_PWM - 90 : (pulseCounter == 1) ? MOTOR_NEUTRAL_PWM + 1 : MOTOR_REVERSE_PWM;
	}

	private void updateWheelsPWM() {
		if (!_irSensors.foundObstacle())
			_pwmFrontWheels = constrain(1.3 * ((MID_PAN_PWM - _pwmPan) / RANGE_PAN_PWM) * RANGE_WHEELS_PWM + CENTER_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
	}

	public double constrain(double input, double min, double max) {
		return (input < min) ? min : (input > max) ? max : input;
	}

	public void reset() {
		_lastPanPWM = _pwmPan = MID_PAN_PWM;
		_pwmTilt = MID_TILT_PWM;
		_lastMotorPWM = _pwmMotor = MOTOR_NEUTRAL_PWM;
		_pwmFrontWheels = CENTER_FRONT_WHEELS_PWM;
	}

	class IRSensors {
		double _sideLeftIR, _sideRightIR, _frontRightIR, _frontLeftIR;

		public boolean foundObstacle() {
			boolean foundObstacle = false;

			if (_frontRightIR > 0.9) {
				_pwmFrontWheels = LEFT_FULL_TURN_WHEELS_PWM;
				// Log.e(_TAG, Double.toString(_frontRightIR));
				foundObstacle = true;
			} else if (_frontLeftIR > 0.9) {
				_pwmFrontWheels = LEFT_FULL_TURN_WHEELS_PWM;
				foundObstacle = true;
			} else if (_sideLeftIR > 1.1) {
				_pwmFrontWheels = RIGHT_FULL_TURN_WHEELS_PWM - 100;
				foundObstacle = true;
			} else if (_sideRightIR > 1.1) {
				_pwmFrontWheels = LEFT_FULL_TURN_WHEELS_PWM - 100;
				foundObstacle = true;
			}
			return foundObstacle;
		}

		public void updateIRSensorsVoltage(float sideLeftIR, float sideRightIR, float frontRightIR, float frontLeftIR) {
			_sideLeftIR = sideLeftIR;
			_sideRightIR = sideRightIR;
			_frontRightIR = frontRightIR;
			_frontLeftIR = frontLeftIR;
		}
	}

	// ------------------------------------------------------------------------------------------------------------------------------------

	Point increment = new Point(0, 0);
	double target_tilt_position = 0.0;

	static final double kD_X = 0.8;// 003901;//018; // Derivative gain (Kd)

	static final int MID_SCREEN_BOUNDARY = 15;

	public boolean updatePanTiltPWM(Point screenCenterPoint, Point currentCenterPoint) {
		boolean reverse = false;
		Point derivativeTerm = new Point(0, 0);

		// --- Set up objects to calculate the error and derivative error
		Point error = new Point(0, 0); // The position error
		Point setpoint = new Point(0, 0);

		setpoint.x = (screenCenterPoint.x - currentCenterPoint.x) * 1.35;
		if ((setpoint.x < -MID_SCREEN_BOUNDARY || setpoint.x > MID_SCREEN_BOUNDARY) && currentCenterPoint.x > 0) {
			if (_lastCenterPoint.x != currentCenterPoint.x) {
				increment.x = setpoint.x * 0.18;
				_lastPanPWM = _pwmPan;
			}
			error.x = (_pwmPan - increment.x);

			derivativeTerm.x = (_pwmPan - _lastPanPWM);

			_lastPanPWM = _pwmPan;

			_pwmPan = error.x - constrain(kD_X * derivativeTerm.x, -9, 9);

			_pwmPan = constrain(_pwmPan, MIN_PAN_PWM, MAX_PAN_PWM);

			// if (_pwmPan >= MAX_PAN_PWM) {
			// reverse = true;
			// _pwmPan = MID_PAN_PWM;
			// }

			_lastCenterPoint.x = currentCenterPoint.x;
		}

		setpoint.y = (currentCenterPoint.y - screenCenterPoint.y) * 0.8;
		if ((setpoint.y < -MID_SCREEN_BOUNDARY || setpoint.y > MID_SCREEN_BOUNDARY) && currentCenterPoint.y > 0) {
			if (_lastCenterPoint.y != currentCenterPoint.y) {
				target_tilt_position = (_pwmTilt - setpoint.y);
				increment.y = setpoint.y * 0.41;
			}
			error.y = (_pwmTilt - increment.y);

			if (target_tilt_position > MID_TILT_PWM && error.y > target_tilt_position && error.y > _pwmTilt) {
				_pwmTilt = target_tilt_position;
				increment.y = 0;
			}
			if (target_tilt_position > MID_TILT_PWM && error.y < target_tilt_position && error.y < _pwmTilt) {
				_pwmTilt = target_tilt_position;
				increment.y = 0;
			} else if (target_tilt_position < MID_TILT_PWM && error.y < target_tilt_position && error.y < _pwmTilt) {
				_pwmTilt = target_tilt_position;
				increment.y = 0;
			} else if (target_tilt_position < MID_TILT_PWM && error.y > target_tilt_position && error.y > _pwmTilt) {
				_pwmTilt = target_tilt_position;
				increment.y = 0;
			} else {
				_pwmTilt = error.y;
			}

			_pwmTilt = constrain(_pwmTilt, MIN_TILT_PWM, MAX_TILT_PWM);

			_lastCenterPoint.y = currentCenterPoint.y;
		}

		return reverse;
	}
}