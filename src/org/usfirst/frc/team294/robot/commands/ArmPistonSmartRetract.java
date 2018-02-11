package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.PistonPositions;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmPistonSmartRetract extends Command {
	boolean pistonFinal;
	boolean retract;
	double destAngle;

	public ArmPistonSmartRetract(double destAngle, boolean pistonFinal) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.protoArmPiston);
		requires(Robot.protoArmMotor);
		this.pistonFinal = pistonFinal;
		this.destAngle = destAngle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		this.retract = false;
		double currentAngle = Robot.protoArmMotor.getArmDegrees();
		double destAngle = Robot.protoArmMotor.getCurrentArmTarget();
		if (RobotMap.getArmZone(currentAngle) == ArmZones.Middle
				|| RobotMap.getArmZone(currentAngle) == ArmZones.Backwards
				|| RobotMap.getArmZone(currentAngle) != RobotMap.getArmZone(destAngle) || !pistonFinal) {
			// If arm is in the middle or backwards zone set pistons to retracted
			Robot.protoArmPiston.setMinor(PistonPositions.Retracted);
			Robot.protoArmPiston.setMajor(PistonPositions.Retracted);
			retract = true;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (!retract)
			return true;
		else if (retract && Robot.protoArmPiston.getMajor() == PistonPositions.Retracted)
			// && Robot.protoArmPiston.getMinor() == PistonPositions.Retracted)
			return true;
		return false; // changed to true for testing with one piston TODO
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
