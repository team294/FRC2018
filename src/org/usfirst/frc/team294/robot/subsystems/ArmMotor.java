package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team294.utilities.*;

/**
 * The subsystem controlling the arm angle (but not the piston)
 */
public class ArmMotor extends Subsystem {

	private final TalonSRX armMotor1 = new TalonSRX(RobotMap.armMotor1);
	private final TalonSRX armMotor2 = new TalonSRX(RobotMap.armMotor2);

	private final double DEGREES_PER_TICK = RobotMap.degreesPerTicks; // Put in robot.preferences or change proto arm to
																		// magnetic encoder
	private final double TICKS_PER_DEGREE = 1.0 / RobotMap.degreesPerTicks;

	private final double MAX_UP_PERCENT_POWER = 0.5; // Up these speeds after testing. 0.8 before
	private final double MAX_DOWN_PERCENT_POWER = -0.3; // -0.5 before
	
	private final double kPu;
	private final double kIu;
	private final double kDu;
	private final double kPd;
	private final double kId;
	private final double kDd;
	private final double kF;
	private double initAngle;
	private double finalAngle;
	private double prevError;
	private double error;
	private double intError;
	private double armMoment = 19.26*16*19.985/519;//11.87;

	// variables to check if arm Encoder is reliable
	private double armEncoderStartValue = getArmEncRaw();
	public boolean joystickControl;
	
	int loop = 0;
	private ArmProfileGenerator trapezoid;
	private double angle;
	public ArmMotor() {
		
		// armMotor.set(ControlMode.Position, 3);
		armMotor1.set(ControlMode.PercentOutput, 0);
		armMotor2.set(ControlMode.Follower, RobotMap.armMotor1);
		armMotor1.setNeutralMode(NeutralMode.Brake);
		armMotor1.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
				0);
		armMotor1.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen,
				0);
		armMotor1.overrideLimitSwitchesEnable(true); // pass false to force disable limit switch

		// Closed-loop control structures
		armMotor1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
		armMotor1.setSensorPhase(false);

		// armMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 0, 0x00, 0x00,
		// 0x00); // Change parameter to 1 for non-continuous
		armMotor1.selectProfileSlot(0, 0);
		//armMotor1.config_kF(0, 0.0, 10);
		//armMotor1.config_kP(0, 3.5, 10); // old term 90 with pot, 4.4 converted to new encoder
		//armMotor1.config_kI(0, 0.0, 10);
		//armMotor1.config_kD(0, 0.0, 10);
		kPu = 0.035;
		kIu = 0.0;
		kDu = 0.02;
		kPd = 0.035;
		kId = 0.005;
		kDd = 0.02;
		kF = 0.2/armMoment;
		armMotor1.configClosedloopRamp(0.25, 10);
		armMotor1.configPeakOutputForward(MAX_UP_PERCENT_POWER, 10);
		armMotor1.configPeakOutputReverse(MAX_DOWN_PERCENT_POWER, 10);
		trapezoid = new ArmProfileGenerator(getArmDegrees(), getArmDegrees(), 0, 0, 0);
	}
	
	/**
	 * 
	 */
	public void startPID(double angle) {
		// TODO: add checks to make sure arm does not go outside of safe areas
		// TODO: integrate checks with piston to avoid penalties for breaking frame
		// perimeter
		initAngle = getArmDegrees();
		finalAngle = angle;
		trapezoid = new ArmProfileGenerator(initAngle, angle,0, 90, 50);
		intError = 0;
		prevError = 0;
		error = 0;
//		double encoderDegrees = angle * TICKS_PER_DEGREE;
//		setArmPositionScaled(encoderDegrees);
		
	}
	
	/**
	 * 
	 */
	
	

	/**
	 * Controls the arm based on Percent VBUS FOR CALIBRATION ONLY DO NOT USE AT
	 * COMPETITION *EVER*
	 * 
	 * @param percent
	 *            voltage, minimum -0.3 and maximum 0.7
	 */
	public void setArmMotorToPercentPower(double percent) {

		SmartDashboard.putNumber("Arm Motor Percent", percent);
		if (percent > MAX_UP_PERCENT_POWER)
			percent = MAX_UP_PERCENT_POWER; // Can be +/- 1 after testing
		if (percent < MAX_DOWN_PERCENT_POWER)
			percent = MAX_DOWN_PERCENT_POWER;
//		if (percent < .1 && percent > -.1) // Need this for joystick deadzone
//			percent = 0;
		armMotor1.set(ControlMode.PercentOutput, percent);
		System.out.println("Arm motor " + armMotor1.getDeviceID() + " set to percent " + percent + ", output "
				+ armMotor1.getMotorOutputVoltage() + " V," + armMotor1.getOutputCurrent() + " A, Bus at "
				+ armMotor1.getBusVoltage() + " V");
	}

	/**
	 * Returns value of encoder on arm, adjusted for zero degree reference at level.
	 * Also updates Smart Dashboard. </br>
	 * The reference value for the arm position at zero degrees is set in the
	 * Shuffleboard network table/preferences section.</br>
	 * <b>To reset:</b> Set the arm to the zero position, set the armCalZero to 0.
	 * The value at that read should then be entered into the armCalZero field.
	 **/
	public double getArmEnc() {
		double encValue = getArmEncRaw() - Robot.robotPrefs.armCalZero;
		SmartDashboard.putNumber("Arm Enc (calibrated)", encValue);
		return (encValue);
	}

	/**
	 * Returns the angle value that the arm is trying to move to in degrees
	 * 
	 * @return desired degree of arm angle
	 */
	public double getCurrentArmTarget() {
		double currTarget = armMotor1.getClosedLoopTarget(0) - Robot.robotPrefs.armCalZero;
		currTarget *= DEGREES_PER_TICK;
		SmartDashboard.putNumber("Desired Angle of Arm in Degrees", currTarget);
		return currTarget;
	}

	/**
	 * Returns the raw value of the encoder on arm, without adjusting for level.
	 * Also updates SmartDashboard. Needed for calibration of 0.
	 * 
	 * @return raw value, probably a large negative number
	 */
	public double getArmEncRaw() {
		double encVal = armMotor1.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("Arm Enc Raw", encVal);
		return encVal;
	}

	/**
	 * Gets the current angle of the arm, in degrees (converted from encoder) </br>
	 * Uses getArmEnc and multiplies by degrees per click constant
	 * 
	 * @return angle in degrees
	 */
	public double getArmDegrees() {
		double armAngle = getArmEnc() * DEGREES_PER_TICK;
		SmartDashboard.putNumber("Arm angle Value", armAngle);
		return (armAngle);
	}


	/**
	 * Increments or decrements the arm by 7 degrees
	 * 
	 * @param increment
	 *            true for increment, false for decrement
	 */
	public void armIncrement(boolean increment) {
		armIncrement(7, increment);
	}

	/**
	 * Increments or decrements the arm
	 * 
	 * @param difference
	 *            the amount to increment/decrement by
	 * @param increment
	 *            true for increment, false for decrement
	 */
	public void armIncrement(int difference, boolean increment) {
		if (increment) {
			if (RobotMap.getArmZone(getArmDegrees()) == RobotMap.getArmZone(getArmDegrees() - difference)) {
				startPID(getArmDegrees() - difference);
			}
		} else {
			if (RobotMap.getArmZone(getArmDegrees()) == RobotMap.getArmZone(getArmDegrees() + difference)) {
				startPID(getArmDegrees() + difference);
			}
		}
	}

	/**
	 * Sets the angle of the arm in degrees
	 * 
	 * @param angle
	 *            desired angle, in degrees
	 */

	/**
	 * Sets the position of the arm based on scaled encoder ticks, and converts to
	 * raw (subtracts position at 0)
	 * 
	 * @param position
	 *            desired position, in encoder ticks
	 */
	private void setArmPositionScaled(double position) {
		if (Robot.robotPrefs.armCalibrated) {
			position += (Robot.robotPrefs.armCalZero); // armZeroDegreesCalibration;
			armMotor1.set(ControlMode.Position, position);
		}
	}

	/**
	 * Sets the (raw) position of the arm based on raw encoder ticks, with no
	 * adjustment for zero calibration
	 * 
	 * @param position
	 *            desired position, in raw encoder ticks
	 */
	private void setArmPositionRaw(double position) {
		if (Robot.robotPrefs.armCalibrated) {
			armMotor1.set(ControlMode.Position, position);
		}
	}

	

	/**
	 * Gets the output voltage of the arm motor.</br>
	 * Also updates smart dashboard with voltage
	 * 
	 * @return voltage
	 */
	public double getOutputVoltage() {
		double voltageLeft = armMotor1.getMotorOutputVoltage();
		SmartDashboard.putNumber("Left Arm Motor Output Voltage", voltageLeft);
		return voltageLeft;
	}

	public void checkEncoder() { // TODO figure out correct minimum voltage
		if (!joystickControl) {
		if (loop == 4) 
			{
		//		System.out.println("Motor Voltage " + getOutputVoltage() + " Current Encoder Value " + getArmEncRaw()
		//		+ " Last Encoder Value " + armEncoderStartValue + " loop " + loop);

				if (getOutputVoltage() >= 5.0) {
					if (getArmEncRaw() <= armEncoderStartValue) {
						setArmMotorToPercentPower(0.0);
						Robot.robotPrefs.armCalibrated = false;
					}
					SmartDashboard.putBoolean("Arm Encoder Working", Robot.robotPrefs.armCalibrated);
				}
				if (getOutputVoltage() <= -3.0) {
					if (getArmEncRaw() >= armEncoderStartValue) {
						setArmMotorToPercentPower(0.0);
						Robot.robotPrefs.armCalibrated = false;
					}
					SmartDashboard.putBoolean("Arm Encoder Working", Robot.robotPrefs.armCalibrated);
				} else {
					Robot.robotPrefs.armCalibrated = true;
				}
				armEncoderStartValue = getArmEncRaw();
			}
		loop = (loop <= 4) ? loop : 0;
		loop++;
		}
	}

	/**
	 * Updates encoder and angle measurements on the SmartDashboard
	 */
	public void updateSmartDashboard() {
		getArmEnc();
		getArmDegrees();
		getOutputVoltage();
		if (armMotor1.getControlMode()== ControlMode.Position) {
			SmartDashboard.putNumber("Arm Motor Error", armMotor1.getClosedLoopError(0));
			SmartDashboard.putNumber("Arm Motor Target", armMotor1.getClosedLoopTarget(0));
		}
	}

	public void periodic() {
		updateSmartDashboard();
		// Set armCalZero, if not already set, by using known value of lower limit
		// switch
		if (!Robot.robotPrefs.armCalibrated) {
			SensorCollection sc = armMotor1.getSensorCollection();
			if (sc.isRevLimitSwitchClosed()) {
				// TODO uncomment and test for possible sign error
				Robot.robotPrefs.setArmCalibration( getArmEncRaw() - (RobotMap.minAngle * TICKS_PER_DEGREE), false);
			}
		}if(DriverStation.getInstance().isEnabled()) {
			trapezoid.updateProfileCalcs();
			error = 	trapezoid.getCurrentPosition() - getArmDegrees();
			intError = intError + error*.02;
			double percentPower = kF*armMoment*Math.cos(Math.toRadians(getArmDegrees()));
			//Gain schedule until stuff gets tuned
			if(finalAngle - initAngle > 0)
				percentPower += kPu * error + ((error-prevError)*kDu)+(kIu*intError);
			else
				percentPower += kPd * error + ((error-prevError)*kDd)+(kId*intError);
			prevError = error;
			SmartDashboard.putNumber("PID Error", error);
			SmartDashboard.putNumber("PID percent power", percentPower);
			SmartDashboard.putNumber("Profile getCurrentPosition", trapezoid.getCurrentPosition());
			setArmMotorToPercentPower(percentPower);
		}else {
			setArmMotorToPercentPower(0);
		}
//		checkEncoder();
		
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new UpdateArmSmartDashboard());
	}

}