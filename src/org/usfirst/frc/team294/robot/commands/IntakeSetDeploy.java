package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeSetDeploy extends Command {

	boolean deployed;
	
    public IntakeSetDeploy(boolean deploy) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	deployed = deploy;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setIntakeDeploy(deployed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return true; // Write a test to see if intake changes state?
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
