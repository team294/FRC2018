package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmPositions;

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
		if (destAngle > RobotMap.maxAngle) destAngle = RobotMap.maxAngle;
    	if (destAngle < RobotMap.minAngle) destAngle = RobotMap.minAngle;
		this.destAngle = destAngle;
	}
	
	public ArmMoveToDestAngle(ArmPositions position) {
    	destAngle = position.getAngle();
    }

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.armMotor.setArmAngle(destAngle);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.armMotor.setArmAngle(destAngle);
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
