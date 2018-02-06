package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

import java.util.concurrent.TimeUnit;

/**
 *
 */
public class CubePickUp extends Command {

	private double leftPercent = -50;
	private double rightPercent = -50;
	private double timeClawClosed = 1000.0;

	public CubePickUp() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.claw);
		// this.leftPercent = leftPercent;
		// this.rightPercent = rightPercent;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.claw.setClawMotorToPercentPower(leftPercent, rightPercent);
		Robot.claw.openClaw();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.inputs.isObjectPresent()) {
			Robot.claw.closeClaw();
			if (timeClawClosed == 1000) {
				timeClawClosed = Timer.getFPGATimestamp();
			} else {
			}
			/*
			 * if (!Robot.inputs.isCubeFullyIn()) { end(); } else { // add a timeOut so
			 * motors don't run for too long }
			 */
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		if (!Robot.inputs.isCubeFullyIn() || Timer.getFPGATimestamp() >= timeClawClosed + 1) {
			end();
			timeClawClosed = 1000;
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
		end();
	}
}