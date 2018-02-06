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
		requires(Robot.protoArmPiston);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		double currentAngle = Robot.protoArmMotor.getArmDegrees();
		if (currentAngle <= RobotMap.lowerBound && currentAngle >= RobotMap.minAngle) {
			Robot.protoArmPiston.extendMinorPiston();
			Robot.protoArmPiston.retractMajorPiston();
		} else if (currentAngle <= RobotMap.upperBound && currentAngle >= RobotMap.middleBound) {
			Robot.protoArmPiston.extendMinorPiston();
			Robot.protoArmPiston.extendMajorPiston();
		} else {
			Robot.protoArmPiston.retractMinorPiston();
			Robot.protoArmPiston.retractMajorPiston();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		double currentAngle = Robot.protoArmMotor.getArmDegrees();
		if (currentAngle <= RobotMap.lowerBound && currentAngle >= RobotMap.minAngle) {
			return (Robot.protoArmPiston.getMajor() == PistonPositions.Retracted)&& (Robot.protoArmPiston.getMinor()== PistonPositions.Extended);
		} else if (currentAngle <= RobotMap.upperBound && currentAngle >= RobotMap.middleBound) {
			return (Robot.protoArmPiston.getMajor() == PistonPositions.Extended)&& (Robot.protoArmPiston.getMinor()== PistonPositions.Extended);
		} else {
			return (Robot.protoArmPiston.getMajor() == PistonPositions.Retracted)&& !(Robot.protoArmPiston.getMinor()== PistonPositions.Retracted);
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
