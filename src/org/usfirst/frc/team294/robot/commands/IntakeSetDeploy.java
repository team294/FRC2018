package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeSetDeploy extends Command {

	boolean deployed;
	boolean waitForMove; 
	
	/**
	 * Deploys or retracts the intake based on parameter
	 * @param deploy true = deployed, false = retracted
	 * does not wait for the intake to move 
	 */
    public IntakeSetDeploy(boolean deploy) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    	deployed = deploy;
    	waitForMove = false; 
    }
    
    /**
     * Deploys or retracts the intake based on parameter
	 * @param deploy true = deployed, false = retracted
     * @param waitForMove true = wait for intake to move, false = don't wait 
     */
    public IntakeSetDeploy(boolean deploy, boolean waitForMove) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    	deployed = deploy;
    	this.waitForMove = waitForMove; 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setIntakeDeploy(deployed);
//    	Robot.log.writeLogEcho("Intake Command," + (deployed ? "deploy" : "retract"));		// intake.setIntakeDeploy() does lots of logging
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if(waitForMove) {
    		return timeSinceInitialized() > 0.5; 
    	}
    	else {
    		return true; // Write a test to see if intake changes state?
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
