package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.OI;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.UpdateArmSmartDashboard;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
**/

/**
 * The subsystem controlling the arm angle (but not the piston)
 */
public class ProtoArmMotor extends Subsystem {

	private final WPI_TalonSRX armMotor1 = new WPI_TalonSRX(RobotMap.armMotor1);
	private final WPI_TalonSRX armMotor2 = new WPI_TalonSRX(RobotMap.armMotor2);

	private final double DEGREES_PER_TICK = RobotMap.degreesPerTicks;
	private final double TICKS_PER_DEGREE = 1.0 / RobotMap.degreesPerTicks;

	private final double MAX_UP_PERCENT_POWER = 0.8;
	private final double MAX_DOWN_PERCENT_POWER = -0.5;

	public ProtoArmMotor() {

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
		armMotor1.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
		armMotor1.setSensorPhase(true);
		// armMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 0, 0x00, 0x00,
		// 0x00); // Change parameter to 1 for non-continuous
		armMotor1.selectProfileSlot(0, 0);
		armMotor1.config_kF(0, 0.0, 10);
		armMotor1.config_kP(0, 90.0, 10); // old term 50, 50 works
		armMotor1.config_kI(0, 0.0, 10);
		armMotor1.config_kD(0, 0.0, 10);
		armMotor1.configClosedloopRamp(0.25, 10);
		armMotor1.configPeakOutputForward(MAX_UP_PERCENT_POWER, 10);
		armMotor1.configPeakOutputReverse(MAX_DOWN_PERCENT_POWER, 10);
	}

	public void calibrate() {
		// yeah we're gonna fill this out later with uhhhhh something
		// Probably won't need something, Shuffleboard and robot prefs can be used
		// instead.
		// No, this will be used for calibrating the arm between robots
		// But you can also use Shuffleboard and Robotprefs to calibrate the arm between
		// different robots, no need for a method to do it
	}

	/**
	 * Controls the arm based on Percent VBUS
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
	 * Returns value of pot on arm, adjusted for zero degree reference at level.
	 * Also updates Smart Dashboard. </br>
	 * The reference value for the arm position at zero degrees is set in the
	 * Shuffleboard network table/preferences section.</br>
	 * <b>To reset:</b> Set the arm to the zero position, set the armCalZero to 0.
	 * The value at that read should then be entered into the armCalZero field.
	 **/
	public double getArmPot() {
		double potValue = getArmPotRaw() - (Robot.armCalZero);// armZeroDegreesCalibration; //Removed zero for testing
																// purposes
		// int potValue = armMotor.getSensorCollection().getAnalogIn();
		SmartDashboard.putNumber("Arm Pot Value", potValue);
		return (potValue);
	}

	/**
	 * Returns the angle value that the arm is trying to move to in degrees
	 * 
	 * @return desired degree of arm angle
	 */
	public double getCurrentArmTarget() {
		double currTarget = armMotor1.getClosedLoopTarget(0) - (Robot.armCalZero);
		currTarget *= DEGREES_PER_TICK;
		SmartDashboard.putNumber("Desired Angle of Arm in Degrees", currTarget);
		return currTarget;
	}

	/**
	 * Returns the raw value of the pot on arm, without adjusting for level. Also
	 * updates SmartDashboard. Needed for calibration of 0.
	 * 
	 * @return raw value, probably a large negative number
	 */
	public double getArmPotRaw() {
		double potVal = armMotor1.getSelectedSensorPosition(0);
		SmartDashboard.putNumber("Arm Pot Raw", potVal);
		return potVal;
	}

	/**
	 * Gets the current angle of the arm, in degrees (converted from potentiometer)
	 * </br>
	 * Uses getArmPot and multiplies by degrees per click constant
	 * 
	 * @return angle in degrees
	 */
	public double getArmDegrees() {
		double armAngle = getArmPot() * DEGREES_PER_TICK;
		SmartDashboard.putNumber("Arm angle Value", armAngle);
		return (armAngle);
	}

	public void armPositionJoystick() { // This is not working and difficult to use. Using armAdjustJoystickButtons is
										// better
		double armAdjustment = OI.armJoystick.getY();
		double currTarget = armMotor1.getClosedLoopTarget(0);
		armAdjustment = (Math.abs(armAdjustment) < 0.1) ? 0.0 : armAdjustment * 1.0; // 1.8 works
		if (currTarget > -150 && armAdjustment > 0)
			armAdjustment = 0;
		if (currTarget < -300 && armAdjustment < 0)
			armAdjustment = 0;
		SmartDashboard.putNumber("Joystick Value", armAdjustment);
		SmartDashboard.putNumber("Target Position", currTarget);
		// currTarget += armAdjustment;
		setArmPositionRaw(currTarget + armAdjustment);
	}

	public void armAdjustJoystickButtonLower() {
		setArmAngle(getArmDegrees() - 7);
	}

	public void armAdjustJoystickButtonRaise() {
		setArmAngle(getArmDegrees() + 7);
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
	 * Sets the position of the arm based on scaled potentiometer ticks, and
	 * converts to raw (subtracts position at 0)
	 * 
	 * @param position
	 *            desired position, in pot ticks
	 */
	private void setArmPositionScaled(double position) {
		position += (Robot.armCalZero); // armZeroDegreesCalibration;
		armMotor1.set(ControlMode.Position, position);
	}

	/**
	 * Sets the (raw) position of the arm based on raw potentiometer ticks, with no
	 * adjustment for zero calibration
	 * 
	 * @param position
	 *            desired position, in scaled pot ticks
	 */
	private void setArmPositionRaw(double position) {
		armMotor1.set(ControlMode.Position, position);
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
	 * Updates pot and angle measurements on the SmartDashboard
	 */
	public void updateSmartDashboard() {
		getArmPot();
		getArmDegrees();
		getOutputVoltage();
		SmartDashboard.putNumber("Arm Motor Error", armMotor1.getClosedLoopError(0));
		SmartDashboard.putNumber("Arm Motor Target", armMotor1.getClosedLoopTarget(0));
	}

	public void periodic() {
		updateSmartDashboard();
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new UpdateArmSmartDashboard());
	}

}