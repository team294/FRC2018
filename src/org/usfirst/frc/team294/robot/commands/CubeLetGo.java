package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubeLetGo extends Command {

	private double leftPercent = 50; // may want to have different speeds
	private double rightPercent = 50;
	private double timeLetGo;

	public CubeLetGo() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.openClaw();
		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.claw.openClaw();
		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
		if (timeLetGo == 1000) {
			timeLetGo = Timer.getFPGATimestamp();
		} else {
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (Timer.getFPGATimestamp() >= timeLetGo + 2) {
			end();
			timeLetGo = 1000;
			return true;
		} else {
			return false;
		}
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.claw.setClawMotorToPercentPower(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		Robot.claw.setClawMotorToPercentPower(0, 0);
	}
}