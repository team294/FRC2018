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
	private double speedScale = 0.0001;
	private boolean success;

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

		success = (distErr < 0) ? true : false;
		if (!success) {
			double encoderDifference = Robot.driveTrainSubsystem.getLeftEncoderPosition()
					- Robot.driveTrainSubsystem.getRightEncoderPosition();
			double leftSpeed = speed;
			double rightSpeed = speed;
			if (encoderDifference < 0) {
				rightSpeed -= speedScale * (Math.abs(encoderDifference));
			} else if (encoderDifference > 0) {
				leftSpeed -= speedScale * (Math.abs(encoderDifference));
			}
			leftSpeed = (leftSpeed < 0) ? 0 : leftSpeed;
			rightSpeed = (rightSpeed < 0) ? 0 : rightSpeed;
			leftSpeed = (leftSpeed > 1) ? 1 : leftSpeed;
			rightSpeed = (rightSpeed > 1) ? 1 : rightSpeed;
			SmartDashboard.putNumber("Left Speed", leftSpeed);
			SmartDashboard.putNumber("Right Speed", rightSpeed);
			Robot.driveTrainSubsystem.tankDrive(leftSpeed, rightSpeed);
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return success;
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
