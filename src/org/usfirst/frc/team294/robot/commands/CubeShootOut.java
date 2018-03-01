package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;
import org.usfirst.frc.team294.robot.RobotMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeShootOut extends Command {

	private double leftPercent = 100; // may want to have different speeds
	private double rightPercent = 100;
	private double timeShot = 0;

	public CubeShootOut() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
		Robot.intake.setIntakeMotorPercent(leftPercent);
		timeShot = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if ((!Robot.claw.getPhotoSwitch() && Timer.getFPGATimestamp() >= timeShot + 2) || Timer.getFPGATimestamp() >= timeShot + 3) {
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
