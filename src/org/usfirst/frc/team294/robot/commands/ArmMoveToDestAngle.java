package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToDestAngle extends Command {

	private double destAngle;

	public ArmMoveToDestAngle() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.protoArmMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		destAngle = (destAngle > RobotMap.Ang4) ? RobotMap.Ang4 : destAngle;
		destAngle = (destAngle < RobotMap.Ang0) ? RobotMap.Ang0 : destAngle;
		Robot.protoArmMotor.setArmAngle(destAngle);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
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
