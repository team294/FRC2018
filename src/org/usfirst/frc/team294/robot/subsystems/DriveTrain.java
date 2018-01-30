package org.usfirst.frc.team294.robot.subsystems;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A Subsystem to control the Drive Train
 */
public class DriveTrain extends Subsystem {
	private final WPI_TalonSRX rightMotor1 = new WPI_TalonSRX(RobotMap.rightMotor1);
	private final WPI_TalonSRX rightMotor2 = new WPI_TalonSRX(RobotMap.rightMotor2);
	private final WPI_TalonSRX rightMotor3 = new WPI_TalonSRX(RobotMap.rightMotor3);
	private final WPI_TalonSRX leftMotor1 = new WPI_TalonSRX(RobotMap.leftMotor1);
	private final WPI_TalonSRX leftMotor2 = new WPI_TalonSRX(RobotMap.leftMotor2);
	private final WPI_TalonSRX leftMotor3 = new WPI_TalonSRX(RobotMap.leftMotor3);
	public final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor2, rightMotor2);
	// NavX. Create the object , in the DriveTrain() constructor, so that we can
	// catch
	// errors.
	private AHRS ahrs;
	private double yawZero = 0;

	// Track left and right encoder zeros here to minimize latency in Talons.
	private double leftEncoderZero = 0, rightEncoderZero = 0;

	public DriveTrain() {// initialize Followers
		// motor2 are the main motors, and motor 1 and 3 are the followers.
		leftMotor1.set(ControlMode.Follower, RobotMap.leftMotor2);
		leftMotor3.set(ControlMode.Follower, RobotMap.leftMotor2);
		rightMotor1.set(ControlMode.Follower, RobotMap.rightMotor2);
		rightMotor3.set(ControlMode.Follower, RobotMap.rightMotor2);
		leftMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		rightMotor2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

		leftMotor2.setSensorPhase(true);
		// leftMotor2.setInverted(true); // This might need to be changed to rightmotor2

		leftMotor2.clearStickyFaults(0);
		rightMotor2.clearStickyFaults(0);
		robotDrive.setSafetyEnabled(false); // Maybe Bad????????? Check Later
		try {
			/* Communicate w/navX MXP via the MXP SPI Bus. */
			/* Alternatively: I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB */
			/*
			 * See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for
			 * details.
			 */

			ahrs = new AHRS(I2C.Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}
		ahrs.zeroYaw();
	}

	/**
	 * Zero the gyro position.
	 */
	public void zeroGyroRoataion() {
		// set yawZero to gryo angle
		yawZero = ahrs.getAngle();
		// System.err.println("PLZ Never Zero the Gyro Rotation it is not good");
	}

	/**
	 * Get the rotation of the gyro. Uses normal people degrees.
	 * 
	 * @return Current angle from 0 to 360 degrees.
	 */
	public double getGyroRotation() {
		double angle = ahrs.getAngle() - yawZero;
		// Angle will be in terms of raw gyro units (-inf,inf), so you need to convert
		// to [0,360]
		angle = angle % 360;
		SmartDashboard.putNumber("NavX Angle", angle);
		return angle;
	}

	public void tankDrive(double powerLeft, double powerRight) {
		this.robotDrive.tankDrive(powerLeft, powerRight);
	}

	/**
	 * Zeros the left encoder position.
	 */
	public void zeroLeftEncoder() {
		leftEncoderZero = leftMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Zeros the right encoder position.
	 */
	public void zeroRightEncoder() {
		rightEncoderZero = rightMotor2.getSelectedSensorPosition(0);
	}

	/**
	 * Get the position of the left encoder
	 * 
	 * @return encoder position
	 */
	public double getLeftEncoderPosition() {
		double position = leftMotor2.getSelectedSensorPosition(0) - leftEncoderZero;
		SmartDashboard.putNumber("Left Encoder Position", position);
		return position;
	}

	/**
	 * Get the position of the right encoder
	 * 
	 * @return encoder position
	 */
	public double getRightEncoderPosition() {
		double position = rightMotor2.getSelectedSensorPosition(0) - rightEncoderZero;
		SmartDashboard.putNumber("Right Encoder Position", position);
		return position;
	}

	/**
	 * Sets the robot to drive at a curve.
	 * 
	 * @param speedPct
	 *            Percent output of motor -1.0 to 1.0
	 * @param curve
	 *            the rate at which the robot will curve -1.0 to 1.0. Clockwise is
	 *            positive.
	 */
	public void driveAtCurve(double speedPct, double curve) {
		robotDrive.curvatureDrive(speedPct, curve, false);
	}

	/**
	 * Set the percent output of the left motor.
	 * 
	 * @param powerPct
	 *            Percent of power -1.0 to 1.0
	 */
	public void setLeftMotors(double speedPct) {
		leftMotor2.set(ControlMode.PercentOutput, -speedPct);
	}

	/**
	 * Set the percent output of the right motor.
	 * 
	 * @param powerPct
	 *            Percent of power -1.0 to 1.0
	 */
	public void setRightMotors(double speedPct) {
		rightMotor2.set(ControlMode.PercentOutput, speedPct);
	}

	/**
	 * Stops all motors.
	 */
	public void stopAllMotors() {
		setLeftMotors(0);
		setRightMotors(0);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new DriveWithJoystick());
	}
}
