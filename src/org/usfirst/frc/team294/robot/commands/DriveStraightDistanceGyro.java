package org.usfirst.frc.team294.robot.commands;

import java.util.function.Supplier;

import org.omg.PortableServer.ImplicitActivationPolicyOperations;
import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.utilities.ToleranceChecker;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightDistanceGyro extends Command {

	private double distErr = 0;
	private double distanceTravel, percentPower;
	private ToleranceChecker tolCheck;
	private boolean success;
	private double distSpeedControl;
	private final double kPdist = 0.06, kDdist = 0.24;
	private double prevDistErr;
	private double angleErr;
	private double intErr = 0;
	private double prevAngleErr;
	private double kPangle = .06;
	private double kIangle = .002;
	private double kDangle = .1;
	private double curve;
	private double minSpeed = .1;
	private double angleBase;

	public DriveStraightDistanceGyro(double distanceTravel, double percentPower) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainSubsystem);
		this.distanceTravel = distanceTravel;
		this.percentPower = percentPower;
	}
	public double encoderTicksToInches(double encoderticks) {
		return (encoderticks / RobotMap.encoderTicksPerRevolution) * RobotMap.wheelCircumference;
	}
	public double inchesToEncoderTicks(double inches) {
		return (inches / RobotMap.wheelCircumference) * RobotMap.encoderTicksPerRevolution;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		distanceTravel = SmartDashboard.getNumber("DistToTravelDSDG", 60);
		distErr = 0;
		prevDistErr = 0;
		angleErr = 0;
		success = false;
		distSpeedControl = 0;
		tolCheck = new ToleranceChecker(1, 5);
		Robot.driveTrainSubsystem.zeroLeftEncoder();
		Robot.driveTrainSubsystem.zeroRightEncoder();
		angleBase = Robot.driveTrainSubsystem.getGyroRotation();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		final double currentDistanceInches = encoderTicksToInches((Robot.driveTrainSubsystem.getLeftEncoderPosition()
				+ Robot.driveTrainSubsystem.getRightEncoderPosition()) / 2.0);

		distErr = distanceTravel - currentDistanceInches;
		SmartDashboard.putNumber("Distance Error", distanceTravel - currentDistanceInches);
		success = tolCheck.success(Math.abs(distanceTravel - currentDistanceInches));
		if (!success) {
			distSpeedControl = distErr * kPdist + (distErr - prevDistErr) * kDdist;
			SmartDashboard.putNumber("SpeedControl", distSpeedControl);
			prevDistErr = distErr;
			distSpeedControl = distSpeedControl > 1 ? 1 : distSpeedControl;
			distSpeedControl = distSpeedControl < -1 ? -1 : distSpeedControl;
			distSpeedControl *= percentPower;
			if (distSpeedControl > 0) {
				distSpeedControl = (distSpeedControl < minSpeed) ? minSpeed : distSpeedControl;
			} else {
				distSpeedControl = (distSpeedControl > -minSpeed) ? -minSpeed : distSpeedControl;
			}
			angleErr = angleBase - Robot.driveTrainSubsystem.getGyroRotation();
			angleErr = (angleErr > 180) ? angleErr - 360 : angleErr;
			intErr = intErr + angleErr * 0.02;
			double dErr = angleErr - prevAngleErr;
			prevAngleErr = angleErr;
			curve = angleErr * kPangle + intErr * kIangle + dErr * kDangle;
			curve = (curve > 0.5) ? 0.5 : curve;
			curve = (curve < -0.5) ? -0.5 : curve;
			curve = (distanceTravel - currentDistanceInches >= 0) ? curve : -curve;
			Robot.driveTrainSubsystem.driveAtCurve(distSpeedControl, curve);

		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return success;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
