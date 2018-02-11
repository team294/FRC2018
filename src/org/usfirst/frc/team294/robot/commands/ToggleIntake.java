package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ToggleIntake extends Command {

	private double leftPercent;
	private double rightPercent;
	private double lastRightPercent = -50;
	private double lastLeftPercent = -50;
	private boolean isIntakeRunning = false;
	
    /**Turns Intake on or off
     * 
     */
	public ToggleIntake() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
	//sets left and right last percent values to the previously used motor power percent
    protected void initialize() {
    	if (Robot.intake.lastLeftPercent != 0) {
    		lastLeftPercent = Robot.intake.lastLeftPercent;
    		lastRightPercent = Robot.intake.lastRightPercent;
    	}
    	if (Robot.intake.lastRightPercent != 0) {
    		lastLeftPercent = Robot.intake.lastLeftPercent;
    		lastRightPercent = Robot.intake.lastRightPercent;
    	}

    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	isIntakeRunning = (isIntakeRunning) ? false : true;
    	leftPercent = (isIntakeRunning) ? lastLeftPercent : 0;
    	rightPercent = (isIntakeRunning) ? lastRightPercent : 0;
    	Robot.intake.setIntakeMotorToPercentPower(leftPercent, rightPercent);
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
