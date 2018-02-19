package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToDestAngle extends Command {

	private double destAngle;

	public ArmMoveToDestAngle(double destAngle) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.armMotor);
		this.destAngle = destAngle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		destAngle = (destAngle > RobotMap.maxAngle) ? RobotMap.maxAngle : destAngle;
		destAngle = (destAngle < RobotMap.minAngle) ? RobotMap.minAngle : destAngle;
		Robot.armMotor.startPID(destAngle);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.armMotor.startPID(destAngle);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// TODO Tolerance Checking
		return true;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
