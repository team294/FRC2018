package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;
import org.usfirst.frc.team294.robot.RobotMap.ArmZones;

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
		
		if ( RobotMap.getArmZone(currentAngle) == ArmZones.Low) {
			if (destAngle >= RobotMap.lowerBound) {
				// We are moving from Low zone to another zone, so move to edge of Low zone
				Robot.protoArmMotor.setArmAngle(RobotMap.lowerBound);
			} else {
				// We aren't leaving Low zone, so just go to destAngle
				Robot.protoArmMotor.setArmAngle(destAngle);
			}
		} else if (RobotMap.getArmZone(currentAngle) == ArmZones.High) {
			if (destAngle >= RobotMap.upperBound) {
				// We are moving from High zone to Backward zone, so move to high edge of High zone
				Robot.protoArmMotor.setArmAngle(RobotMap.upperBound);
			} else if (destAngle <= RobotMap.middleBound) {
				// We are moving from High zone to a lower zone, so move to low edge of High zone
				Robot.protoArmMotor.setArmAngle(RobotMap.middleBound);
			}
			else {
				// We aren't leaving High zone, so just go to destAngle				
				Robot.protoArmMotor.setArmAngle(destAngle);
			}
		} else if(RobotMap.getArmZone(currentAngle) == ArmZones.Backwards) {
			// We are in Middle or Backwards zone, so just go to the dest angle				
			if(RobotMap.getArmZone(destAngle) == ArmZones.Backwards || RobotMap.getArmZone(destAngle) == ArmZones.High) {
				Robot.protoArmMotor.setArmAngle(destAngle);
			}
			else {
				Robot.protoArmMotor.setArmAngle(RobotMap.middleBound);
			}
		} else if(RobotMap.getArmZone(currentAngle) == ArmZones.Middle) {
			// We are in Middle Zone 
			if (!(RobotMap.getArmZone(destAngle) == ArmZones.Backwards)) {
				Robot.protoArmMotor.setArmAngle(destAngle);
			} else {
				Robot.protoArmMotor.setArmAngle(RobotMap.upperBound);
			}
			
		}
		
			
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

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
