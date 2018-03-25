package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeShootOut extends Command {

	//TODO fix this!!!!  Perecents should be -1 to +1. 
	private double leftPercent = 66; // may want to have different speeds
	private double rightPercent = 66;
	private double timeShot = 0;

	public CubeShootOut() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
		requires(Robot.intake);
	}

	/**
	 * Shoot cube at given power
	 * @param percentPower -1 to +1, + to shoot out
	 */
	public CubeShootOut(double percentPower) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
		requires(Robot.intake);
		leftPercent = percentPower;
		rightPercent = percentPower;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
		Robot.intake.setIntakeMotorPercent(-leftPercent);
		timeShot = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if ((!Robot.claw.getPhotoSwitch() && Timer.getFPGATimestamp() >= timeShot + 1) || Timer.getFPGATimestamp() >= timeShot + 3) {
			return true;
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.claw.setClawMotorToPercentPower(0, 0);
		//sets intake motor back to its most recent non-zero, inward speed
		Robot.intake.setIntakeMotorPercent(RobotMap.intakePercentOut);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
