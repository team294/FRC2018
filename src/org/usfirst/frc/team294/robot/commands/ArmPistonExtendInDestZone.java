package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonExtendInDestZone extends Command {

	private double currentAng = Robot.protoArmMotor.getArmDegrees();
	private double destAng;
	private boolean extendPistonAtEnd; // true = extend, false = retract
	private boolean done = false;

	public ArmPistonExtendInDestZone(boolean extendPistonAtEnd, double destAng) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.protoArmPiston);
		this.extendPistonAtEnd = extendPistonAtEnd;
		this.destAng = destAng;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (!extendPistonAtEnd || !((destAng < RobotMap.Ang3 && destAng > RobotMap.Ang2)
				|| (destAng < RobotMap.Ang1 && destAng > RobotMap.Ang0))) {
			done = true;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (destAng < RobotMap.Ang3 && destAng > RobotMap.Ang2) {
			if (currentAng < RobotMap.Ang3 && currentAng > RobotMap.Ang2) {
				Robot.protoArmPiston.extendMajorPiston();
				done = true;
			}
		} else if (destAng < RobotMap.Ang1 && destAng > RobotMap.Ang0) {
			if (currentAng < RobotMap.Ang1 && currentAng > RobotMap.Ang0) {
				Robot.protoArmPiston.extendMajorPiston();
				done = true;
			}
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return done;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
