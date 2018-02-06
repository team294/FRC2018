package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmMoveToEdge extends Command {
	private double destAngle;
	private double currentAngle;

	public ArmMoveToEdge(double destAngle) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.protoArmMotor);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		currentAngle = Robot.protoArmMotor.getArmDegrees();
		if ((currentAngle >= RobotMap.minAngle) && (currentAngle <= RobotMap.lowerBound)) {
			if (destAngle >= RobotMap.lowerBound) {
				Robot.protoArmMotor.setArmAngle(RobotMap.lowerBound);
			} else
				Robot.protoArmMotor.setArmAngle(destAngle);
		} else if ((currentAngle >= RobotMap.middleBound) && (currentAngle <= RobotMap.upperBound)) {
			if (destAngle >= RobotMap.upperBound) {
				Robot.protoArmMotor.setArmAngle(RobotMap.upperBound);
			} else if (destAngle <= RobotMap.middleBound)
				Robot.protoArmMotor.setArmAngle(RobotMap.middleBound);
			else
				Robot.protoArmMotor.setArmAngle(destAngle);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		currentAngle = Robot.protoArmMotor.getArmDegrees();
		if ((currentAngle >= RobotMap.minAngle) && (currentAngle <= RobotMap.lowerBound)) {
			if (destAngle >= RobotMap.lowerBound) {
				Robot.protoArmMotor.setArmAngle(RobotMap.lowerBound);
			} else
				Robot.protoArmMotor.setArmAngle(destAngle);
		} else if ((currentAngle >= RobotMap.middleBound) && (currentAngle <= RobotMap.upperBound)) {
			if (destAngle >= RobotMap.upperBound) {
				Robot.protoArmMotor.setArmAngle(RobotMap.upperBound);
			} else if (destAngle <= RobotMap.middleBound)
				Robot.protoArmMotor.setArmAngle(RobotMap.middleBound);
			else
				Robot.protoArmMotor.setArmAngle(destAngle);
		}
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
