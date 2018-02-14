package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmRetractMajorPiston extends Command {


	public ArmRetractMajorPiston() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.armPiston);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.armPiston.setMajor(PistonPositions.Retracted);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.armPiston.getMajor() == RobotMap.PistonPositions.Retracted;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
