package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPiston10InchRetract extends Command {

	private double currentAng = Robot.protoArmMotor.getArmDegrees();
	private boolean extendPistonAtEnd; // true = extend, false = retract

	public ArmPiston10InchRetract(boolean extendPistonAtEnd) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.protoArmPiston);
		this.extendPistonAtEnd = extendPistonAtEnd;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (currentAng >= RobotMap.Ang3 || currentAng <= RobotMap.Ang2 || !extendPistonAtEnd) {
			Robot.protoArmPiston.retractMajorPiston();
		}

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.protoArmPiston.getMajor() == RobotMap.PistonPositions.Retracted;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
