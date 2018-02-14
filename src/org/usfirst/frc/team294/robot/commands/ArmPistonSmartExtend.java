package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonSmartExtend extends Command {

	public ArmPistonSmartExtend() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.armPiston);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		double currentAngle = Robot.armMotor.getArmDegrees();
		if (currentAngle <= RobotMap.lowerBound && currentAngle >= RobotMap.minAngle) {
			Robot.armPiston.setMinor(PistonPositions.Extended);
			Robot.armPiston.setMajor(PistonPositions.Retracted);
		} else if (currentAngle <= RobotMap.upperBound && currentAngle >= RobotMap.middleBound) {
			Robot.armPiston.setMinor(RobotMap.PistonPositions.Extended);
			Robot.armPiston.setMajor(RobotMap.PistonPositions.Extended);
		} else {
			Robot.armPiston.setMinor(PistonPositions.Retracted);
			Robot.armPiston.setMajor(PistonPositions.Retracted);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute
	protected boolean isFinished() {
		double currentAngle = Robot.armMotor.getArmDegrees();
		if (currentAngle <= RobotMap.lowerBound && currentAngle >= RobotMap.minAngle) {
			return (Robot.armPiston.getMajor() == PistonPositions.Retracted)
					&& (Robot.armPiston.getMinor() == PistonPositions.Extended);
		} else if (currentAngle <= RobotMap.upperBound && currentAngle >= RobotMap.middleBound) {
			return (Robot.armPiston.getMajor() == PistonPositions.Extended)
					&& (Robot.armPiston.getMinor() == PistonPositions.Extended);
		} else {
			return (Robot.armPiston.getMajor() == PistonPositions.Retracted)
					&& !(Robot.armPiston.getMinor() == PistonPositions.Retracted);
		}
	}

	// Called once after isFinished returns true
	protected void end() {

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
