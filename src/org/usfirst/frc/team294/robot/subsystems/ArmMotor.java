package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import edu.wpi.first.wpilibj.Preferences;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The subsystem controlling the arm angle (but not the piston)
 */
public class ArmMotor extends Subsystem {

	private final TalonSRX armMotor1 = new TalonSRX(RobotMap.armMotor1);
	private final TalonSRX armMotor2 = new TalonSRX(RobotMap.armMotor2);

	private final double DEGREES_PER_TICK = RobotMap.degreesPerTicks; // Put in robot.preferences or change proto arm to
																		// magnetic encoder
	private final double TICKS_PER_DEGREE = 1.0 / RobotMap.degreesPerTicks;

	private final double MAX_UP_PERCENT_POWER = 0.8; // Up these speeds after testing
	private final double MAX_DOWN_PERCENT_POWER = -0.5;

	private boolean armCalibrated = false;  // Default to arm being uncalibrated.  Calibrate from robot preferences, 
											// "Calibrate arm zero position" button on dashboard,
											// or autocal on low limit switch (see periodic() below)
	//private double armCalZero; // Arm encoder position at O degrees (i.e. the calibration factor)

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
		if(Robot.prototypeRobot) armMotor1.setSensorPhase(true);
		else armMotor1.setSensorPhase(false);

		// armMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 0, 0x00, 0x00,
		// 0x00); // Change parameter to 1 for non-continuous
		armMotor1.selectProfileSlot(0, 0);
		armMotor1.config_kF(0, 0.0, 10);
		armMotor1.config_kP(0, 4.4, 10); // old term 90  
		armMotor1.config_kI(0, 0.0, 10);
		armMotor1.config_kD(0, 0.0, 10);
		armMotor1.configClosedloopRamp(0.25, 10);
		armMotor1.configPeakOutputForward(MAX_UP_PERCENT_POWER, 10);
		armMotor1.configPeakOutputReverse(MAX_DOWN_PERCENT_POWER, 10);
	}

	/**
	 * Sets arm angle calibration factor and enables angle control modes for arm.
	 * 
	 * @param armCalZero
	 *            Calibration factor for arm
	 * @param writeCalToPreferences
	 *            true = store calibration in Robot Preferences, false = don't
	 *            change Robot Preferences
	 */

	public void setArmCalibration(double armCalZero, boolean writeCalToPreferences) {
		Robot.robotPrefs.armCalZero = armCalZero;
		armCalibrated = true;
		SmartDashboard.putBoolean("Arm Calibrated", armCalibrated);
		if (writeCalToPreferences) {
			Preferences robotPrefs = Preferences.getInstance();
			robotPrefs.putDouble("calibrationZeroDegrees", armCalZero);
		}
	}

	/**
	 * Controls the arm based on Percent VBUS FOR CALIBRATION ONLY DO NOT USE AT
	 * COMPETITION *EVER*
	 * 
	 * @param percent
	 *            voltage, minimum -0.3 and maximum 0.7
	 */
	public void setArmMotorToPercentPower(double percent) {
		if (percent > MAX_UP_PERCENT_POWER)
			percent = MAX_UP_PERCENT_POWER; // Can be +/- 1 after testing
		if (percent < MAX_DOWN_PERCENT_POWER)
			percent = MAX_DOWN_PERCENT_POWER;
		if (percent < .1 && percent > -.1)
			percent = 0;
		armMotor1.set(ControlMode.PercentOutput, percent);
		System.out.println("Arm motor " + armMotor1.getDeviceID() + " set to percent " + percent + ", output "
				+ armMotor1.getMotorOutputVoltage() + " V," + armMotor1.getOutputCurrent() + " A, Bus at "
				+ armMotor1.getBusVoltage() + " V");
		SmartDashboard.putNumber("Arm Motor Percent", percent);
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
	 * Returns the raw value of the encoder on arm, without adjusting for level. Also
	 * updates SmartDashboard. Needed for calibration of 0.
	 * 
	 * @return raw value, probably a large negative number
	 */
	public double getArmEncRaw() {
		double encVal = armMotor1.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("Arm Enc Raw", encVal);
		return encVal;
	}

	/**
	 * Gets the current angle of the arm, in degrees (converted from encoder)
	 * </br>
	 * Uses getArmEnc and multiplies by degrees per click constant
	 * 
	 * @return angle in degrees
	 */
	public double getArmDegrees() {
		double armAngle = getArmEnc() * DEGREES_PER_TICK;
		SmartDashboard.putNumber("Arm angle Value", armAngle);
		return (armAngle);
	}

	/*
	public void armAdjustJoystickButtonLower() {
		if (RobotMap.getArmZone(getArmDegrees()) == RobotMap.getArmZone(getArmDegrees() - 7)) {
			setArmAngle(getArmDegrees() - 7);
		}
	}

	public void armAdjustJoystickButtonRaise() {
		if (RobotMap.getArmZone(getArmDegrees()) == RobotMap.getArmZone(getArmDegrees() + 7)) {
			setArmAngle(getArmDegrees() + 7);
		}
	}*/
	
	/**
	 * Increments or decrements the arm by 7 degrees
	 * @param increment true for increment, false for decrement
	 */
	public void armIncrement(boolean increment) {
		armIncrement(7, increment);
	}
	
	/**
	 * Increments or decrements the arm
	 * @param difference the amount to increment/decrement by
	 * @param increment true for increment, false for decrement
	 */
	public void armIncrement(int difference, boolean increment) {
		if (increment) {
			if(RobotMap.getArmZone(getArmDegrees()) == RobotMap.getArmZone(getArmDegrees()-difference)){
				setArmAngle(getArmDegrees() - difference);
			}
		} else {
			if(RobotMap.getArmZone(getArmDegrees()) == RobotMap.getArmZone(getArmDegrees()+difference)) {
				setArmAngle(getArmDegrees() + difference);
			}
		}
	}

	/**
	 * Sets the angle of the arm in degrees
	 * 
	 * @param angle
	 *            desired angle, in degrees
	 */
	public void setArmAngle(double angle) {
		// TODO: add checks to make sure arm does not go outside of safe areas
		// TODO: integrate checks with piston to avoid penalties for breaking frame
		// perimeter
		double encoderDegrees = angle * TICKS_PER_DEGREE;
		setArmPositionScaled(encoderDegrees);
	}

	/**
	 * Sets the position of the arm based on scaled encoder ticks, and
	 * converts to raw (subtracts position at 0)
	 * 
	 * @param position
	 *            desired position, in encoder ticks
	 */
	private void setArmPositionScaled(double position) {
		if (armCalibrated) {
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
		if (armCalibrated) {
			armMotor1.set(ControlMode.Position, position);
		}
	}

	/**
	 * Sets the position of the arm in degrees according to the smart dashboard
	 */
	public void setArmFromSmartDashboard() {
		double angle = SmartDashboard.getNumber("set Arm Angle", 0);
		setArmAngle(angle); // For closed-loop testing purposes, passing in encoder values instead of angles
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

	
	/**
	 * Updates encoder and angle measurements on the SmartDashboard
	 */
	public void updateSmartDashboard() {
		getArmEnc();
		getArmDegrees();
		getOutputVoltage();
		SmartDashboard.putNumber("Arm Motor Error", armMotor1.getClosedLoopError(0));
		SmartDashboard.putNumber("Arm Motor Target", armMotor1.getClosedLoopTarget(0));
	}

	public void periodic() {
		updateSmartDashboard();
		// Set armCalZero, if not already set, by using known value of lower limit switch
		if (!armCalibrated) {
			SensorCollection sc = armMotor1.getSensorCollection();
			if (sc.isRevLimitSwitchClosed()) {
				// TODO uncomment and test for possible sign error
				//setArmCalibration( getArmEncRaw() - (RobotMap.minAngle * TICKS_PER_DEGREE), false);
			}
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new UpdateArmSmartDashboard());
	}

}