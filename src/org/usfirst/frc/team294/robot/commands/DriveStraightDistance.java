package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.utilities.ToleranceChecker;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightDistance extends Command {

	private double speed;
	private double encoderTickPerRotation = 4096.0;
	private double wheelCircumference = 4 * Math.PI;
	private double distance;
	private double distErr;
	private ToleranceChecker tolChecker;

	public double encoderTickToInches(double encoderticks) {
		return (encoderticks / encoderTickPerRotation) * wheelCircumference;
	}

	public double inchesToEncoderTicks(double inches) {
		return (inches / wheelCircumference) * encoderTickPerRotation;
	}

	public DriveStraightDistance(double inches, double powerPct) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrainSubsystem);
		tolChecker = new ToleranceChecker(0.5, 5); // 0.5 is in inches
		speed = powerPct;
		distance = (inches / wheelCircumference) * encoderTickPerRotation;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrainSubsystem.zeroRightEncoder();
		Robot.driveTrainSubsystem.zeroLeftEncoder();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		/*
		 * speed = .4-(.35*(Robot.driveTrainSubsystem.getRightEncoderPosition()*1.0 /
		 * (encoderTickPerRotation*9.0)));
		 * Robot.driveTrainSubsystem.setRightMotors(speed);
		 * Robot.driveTrainSubsystem.setLeftMotors(speed);
		 * SmartDashboard.putNumber("Encoder Value Left Rotations",
		 * Robot.driveTrainSubsystem.getRightEncoderPosition()/encoderTickPerRotation);
		 * SmartDashboard.putNumber("Speed", speed);
		 */
		double currentDistance = (Robot.driveTrainSubsystem.getLeftEncoderPosition()
				+ Robot.driveTrainSubsystem.getRightEncoderPosition()) / 2;
		distErr = distance - currentDistance;
		encoderTickToInches(distErr);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Math.abs(Robot.driveTrainSubsystem.getRightEncoderPosition()) > distance; // ((distanceNeeded/(wheelCircumference*gearRatio))*4096);
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrainSubsystem.stopAllMotors();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
