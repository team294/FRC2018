package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ArmIncrementAngle extends Command {

	int difference;
	boolean increment;
	
    public ArmIncrementAngle(boolean increment) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.armMotor);
    	difference = 7;
    	this.increment = increment;
    }
    
    public ArmIncrementAngle(int difference, boolean increment) {
    	requires(Robot.armMotor);
    	this.difference = difference;
    	this.increment = increment;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.armMotor.armIncrement(difference, increment);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}