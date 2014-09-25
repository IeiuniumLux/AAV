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

	public static final int MIN_PAN_PWM = 600;
	public static final int MAX_PAN_PWM = 2500;
	public static final int MIN_TILT_PWM = 1400;
	public static final int MAX_TILT_PWM = 2250;

	public static final int MID_PAN_PWM = (MAX_PAN_PWM + MIN_PAN_PWM) / 2;
	public static final int MID_TILT_PWM = 1800;//(MAX_TILT_PWM + MIN_TILT_PWM) / 2;
	
	public static final int RANGE_PAN_PWM = MAX_PAN_PWM - MID_PAN_PWM;
	
	public static final int RIGHT_FULL_TURN_WHEELS_PWM = 1200;
	public static final int LEFT_FULL_TURN_WHEELS_PWM = 1750;
	public static final int CENTER_FRONT_WHEELS_PWM = (LEFT_FULL_TURN_WHEELS_PWM + RIGHT_FULL_TURN_WHEELS_PWM) / 2;
	public static final int DIF_FRONT_WHEELS_PWM = LEFT_FULL_TURN_WHEELS_PWM - RIGHT_FULL_TURN_WHEELS_PWM; // difference between the side and the middle of the wheel
	
	public static final int RANGE_WHEELS_PWM = LEFT_FULL_TURN_WHEELS_PWM - CENTER_FRONT_WHEELS_PWM;

	public static final int MOTOR_FORWARD_PWM = 1578; 
	public static final int MOTOR_REVERSE_PWM = 1420;
	public static final int MOTOR_NEUTRAL_PWM = 1500;
	
	public static final int MAX_NEUTRAL_CONTOUR_AREA = 1800;
	public static final int MIN_NEUTRAL_CONTOUR_AREA = 600;

	public double pwm_pan;
	public double pwm_tilt;
	public double pwm_motor;
	public double pwm_front_wheels;

	IRSensors ir_sensors;
	
	double last_pan_pwm;
	double last_motor_pwm;
	int pulse_counter = 0;
	boolean was_moving = false;
	
	Point last_center = new Point(0, 0);

	public ActuatorController() {
		// set the pulse width to be exactly the middle
		last_pan_pwm = pwm_pan = MID_PAN_PWM;
		pwm_tilt = MID_TILT_PWM;
		last_motor_pwm = pwm_motor = MOTOR_NEUTRAL_PWM;
		pwm_front_wheels = CENTER_FRONT_WHEELS_PWM;

		ir_sensors = new IRSensors();
	}	
	
	public synchronized double[] getPWMValues() {
		return new double[] {pwm_pan, pwm_tilt, pwm_motor, pwm_front_wheels};
	}
	
	public void updateMotorPWM(double currentContourArea) throws InterruptedException {
		updateWheelsPWM();
		if (currentContourArea > MIN_NEUTRAL_CONTOUR_AREA && currentContourArea < MAX_NEUTRAL_CONTOUR_AREA) {
			// The ESC is intelligent enough to see this as braking.
			if (last_motor_pwm == MOTOR_FORWARD_PWM && was_moving) {
				pwm_motor = MOTOR_REVERSE_PWM - 200;
			} else if (last_motor_pwm == MOTOR_REVERSE_PWM && was_moving) {
				pwm_motor = MOTOR_FORWARD_PWM;
			} else {
					pwm_motor = MOTOR_NEUTRAL_PWM;
			}
			was_moving = false;
		} else if (currentContourArea < MIN_NEUTRAL_CONTOUR_AREA) {
			pwm_motor = MOTOR_FORWARD_PWM;
			was_moving = true;
		} else if (currentContourArea > MAX_NEUTRAL_CONTOUR_AREA) {
			if (last_motor_pwm == MOTOR_NEUTRAL_PWM && !was_moving) {
				pulse_counter = 2;
			}
			pwm_motor = reverseSequence(pulse_counter);
			if (pulse_counter > 0)
				pulse_counter--;
			was_moving = true;			
		}
		last_motor_pwm = pwm_motor;
	}
	
	private int reverseSequence(int pulseCounter) {
		return (pulseCounter == 2) ?  MOTOR_REVERSE_PWM - 100 : (pulseCounter == 1) ? MOTOR_NEUTRAL_PWM + 1 : MOTOR_REVERSE_PWM;
	}

	private void updateWheelsPWM() {
		if (ir_sensors.checkIRSensors())
			pwm_front_wheels = constrain(1.3 * ((MID_PAN_PWM - pwm_pan) / RANGE_PAN_PWM) * RANGE_WHEELS_PWM + CENTER_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
	}

	public double constrain(double input, double min, double max) {
		return (input < min) ? min : (input > max) ? max : input;
	}

	public void reset() {
		last_pan_pwm = pwm_pan = MID_PAN_PWM;
		pwm_tilt = MID_TILT_PWM;
		last_motor_pwm = pwm_motor = MOTOR_NEUTRAL_PWM;
		pwm_front_wheels = CENTER_FRONT_WHEELS_PWM;
	}

	
	public class IRSensors {
		double _frontLeftIRVoltage, _frontRightIRVoltage, _leftSideIRVoltage, _rightSideIRVoltage;
		
		public synchronized boolean isBacking(float frontLeftIRVoltage, float frontRightIRVoltage, float leftSideIRVoltage, float rightSideIRVoltage, double currentContourArea) {
			_frontLeftIRVoltage = frontLeftIRVoltage;
			_frontRightIRVoltage = frontRightIRVoltage;
			_leftSideIRVoltage = leftSideIRVoltage;
			_rightSideIRVoltage = rightSideIRVoltage;
			
			if (currentContourArea > MAX_NEUTRAL_CONTOUR_AREA) {
				was_moving = false;
				return false;
			}
			
			if (_frontLeftIRVoltage > 1.1 && _frontRightIRVoltage > 1.1) {
				last_motor_pwm = pwm_motor = MOTOR_NEUTRAL_PWM;
				return true;
			}
			return false;
		}
		
		
		public boolean checkIRSensors() {
			if (_frontLeftIRVoltage > 1.1) {
				pwm_front_wheels = RIGHT_FULL_TURN_WHEELS_PWM;
				return false;
			} else if (_frontRightIRVoltage > 1.1) {
				pwm_front_wheels = LEFT_FULL_TURN_WHEELS_PWM;
				return false;
			} else 
				if (_leftSideIRVoltage > 1.5) {
				pwm_front_wheels = constrain(pwm_front_wheels - (_leftSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
				return false;
			} else if (_rightSideIRVoltage > 1.5) {
				pwm_front_wheels = constrain(pwm_front_wheels + (_rightSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
				return false;
			}
			return true;
		}

//		int ir_task()
//		{
//		    extern LAYER ir;                       // C structure for task output
//
//		    int detect = read_ir_sensors();        // read sensors
//		    if (detect == LEFT) {                  // if reflection on the left 
//		            ir.cmd = HALF_SPEED;           // request slow down
//		            ir.arg = RIGHT_TURN;           // and turn right
//		            ir.flag = TRUE;                // signal arbitrator we want control
//		    } else {
//		        if (detect == RIGHT) {             // else if reflection on the right
//		            ir.cmd = HALF_SPEED;           // request slow down
//		            ir.arg = LEFT_TURN;            // and turn left
//		            ir.flag = TRUE;                // tell arbitrator we want control
//		        } else {
//		            if (detect == BOTH) {          // else if reflection left and right
//		                ir.cmd = ZERO_SPEED;       // request slow to zero
//		                ir.arg = keep_turning();   // keep turning same direction
//		                ir.flag = TRUE;            // signal arbitrator we want control
//		            } else {
//		               ir.flag = FALSE;            // else no detection, release control
//		            }
//		        }     
//		    }
//		}
		
		
//		public void checkFrontIRSensors() {
//			if (_frontLeftIRVoltage > 1.5) {
//				pwm_front_wheels = constrain(pwm_front_wheels - (_frontLeftIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//				Log.e("<****** _frontLeftIRVoltage", String.valueOf(_frontLeftIRVoltage));
//			}
//			if (_frontRightIRVoltage > 1.5) {
//				pwm_front_wheels = constrain(pwm_front_wheels + (_frontRightIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM, RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//				
//				Log.e("_frontRightIRVoltage *****>", String.valueOf(_frontRightIRVoltage));
//			}
//		}
//
//		public void checkSideIRSensors() {
//			if (_leftSideIRVoltage > 1.5)
//				pwm_front_wheels = constrain(pwm_front_wheels - (0.33 * (_leftSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM), RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//			if (_rightSideIRVoltage > 1.5)
//				pwm_front_wheels = constrain(pwm_front_wheels + (0.33 * (_rightSideIRVoltage / 2.0) * DIF_FRONT_WHEELS_PWM), RIGHT_FULL_TURN_WHEELS_PWM, LEFT_FULL_TURN_WHEELS_PWM);
//		}

		public void updateIRSensorsVoltage(float frontLeftIRVoltage, float frontRightIRVoltage, float leftSideIRVoltage, float rightSideIRVoltage) {
			_frontLeftIRVoltage = frontLeftIRVoltage;
			_frontRightIRVoltage = frontRightIRVoltage;
			_leftSideIRVoltage = leftSideIRVoltage;
			_rightSideIRVoltage = rightSideIRVoltage;
		}
	}

	// ------------------------------------------------------------------------------------------------------------------------------------
	
	Point increment = new Point(0, 0);
	double target_tilt_position = 0.0;

	static final double kD_X = 0.8;// 003901;//018; // Derivative gain (Kd)

	static final int MID_SCREEN_BOUNDARY = 15;

	public void updatePanTiltPWM(Point screenCenterPoint, Point currentCenterPoint) {
		Point derivativeTerm = new Point(0, 0);

		// --- Set up objects to calculate the error and derivative error
		Point error = new Point(0, 0); // The position error
		Point setpoint = new Point(0, 0);

		setpoint.x = (screenCenterPoint.x - currentCenterPoint.x) * 1.3;
		if ((setpoint.x < -MID_SCREEN_BOUNDARY || setpoint.x > MID_SCREEN_BOUNDARY) && currentCenterPoint.x > 0) {
			if (last_center.x != currentCenterPoint.x) {
				increment.x = setpoint.x * 0.2;
				last_pan_pwm = pwm_pan;
			}
			error.x = (pwm_pan - increment.x);

			derivativeTerm.x = (pwm_pan - last_pan_pwm);

			last_pan_pwm = pwm_pan;

			pwm_pan = error.x - constrain(kD_X * derivativeTerm.x, -9, 9);

			pwm_pan = constrain(pwm_pan, MIN_PAN_PWM, MAX_PAN_PWM);

			last_center.x = currentCenterPoint.x;
		}

		setpoint.y = (currentCenterPoint.y - screenCenterPoint.y) * 0.8;
		if ((setpoint.y < -MID_SCREEN_BOUNDARY || setpoint.y > MID_SCREEN_BOUNDARY) && currentCenterPoint.y > 0) {
			if (last_center.y != currentCenterPoint.y) {
				target_tilt_position = (pwm_tilt - setpoint.y);
				increment.y = setpoint.y * 0.41;
			}
			error.y = (pwm_tilt - increment.y);

			if (target_tilt_position > MID_TILT_PWM && error.y > target_tilt_position && error.y > pwm_tilt) {
				pwm_tilt = target_tilt_position;
				increment.y = 0;
			}
			if (target_tilt_position > MID_TILT_PWM && error.y < target_tilt_position && error.y < pwm_tilt) {
				pwm_tilt = target_tilt_position;
				increment.y = 0;
			} else if (target_tilt_position < MID_TILT_PWM && error.y < target_tilt_position && error.y < pwm_tilt) {
				pwm_tilt = target_tilt_position;
				increment.y = 0;
			} else if (target_tilt_position < MID_TILT_PWM && error.y > target_tilt_position && error.y > pwm_tilt) {
				pwm_tilt = target_tilt_position;
				increment.y = 0;
			} else {
				pwm_tilt = error.y;
			}

			pwm_tilt = constrain(pwm_tilt, MIN_TILT_PWM, MAX_TILT_PWM);

			last_center.y = currentCenterPoint.y;
		}
	}
}