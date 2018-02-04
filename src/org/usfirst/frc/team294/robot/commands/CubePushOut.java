package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class CubePushOut extends Command {

	private double leftPercent = 50; // may want to have different speeds
	private double rightPercent = 50;
	
    public CubePushOut() {
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
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
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
