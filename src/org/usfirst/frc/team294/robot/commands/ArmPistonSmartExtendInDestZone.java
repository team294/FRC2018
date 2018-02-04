package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonSmartExtendInDestZone extends Command {

	private double currentAng = Robot.protoArmMotor.getArmDegrees();
	private double destAng;
	private boolean done = false;

	public ArmPistonSmartExtendInDestZone(double destAng) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.protoArmPiston);
		this.destAng = destAng;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (!((destAng < RobotMap.upperBound && destAng > RobotMap.middleBound)
				|| (destAng < RobotMap.lowerBound && destAng > RobotMap.minAngle))) {
			done = true;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (destAng < RobotMap.upperBound && destAng > RobotMap.middleBound) {
			if (currentAng < RobotMap.upperBound && currentAng > RobotMap.middleBound) {
				Robot.protoArmPiston.extendMinorPiston();
				Robot.protoArmPiston.extendMajorPiston();
				done = true;
			}
		} else if (destAng < RobotMap.lowerBound && destAng > RobotMap.minAngle) {
			if (currentAng < RobotMap.lowerBound && currentAng > RobotMap.minAngle) {
				Robot.protoArmPiston.extendMinorPiston();
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
