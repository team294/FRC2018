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
		requires(Robot.armPiston);
		this.pistonFinal = pistonFinal;
		this.destAngle = destAngle;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		this.retract = false;
		ArmZones currentZone = RobotMap.getArmZone(Robot.armMotor.getArmDegrees());
		ArmZones destZone = RobotMap.getArmZone(destAngle);
		if (currentZone == ArmZones.Middle || currentZone == ArmZones.Backwards 
				|| currentZone != destZone || !pistonFinal) {
			// If arm is in the middle or backwards zone set pistons to retracted
			Robot.armPiston.smartRetract();
			retract = true;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (!retract)
			return true;
		else if ( Robot.armPiston.getMajor() == PistonPositions.Retracted
			 && Robot.armPiston.getMinor() == PistonPositions.Retracted)
			return true;
		return false; // According to Liam, the todo is done
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}