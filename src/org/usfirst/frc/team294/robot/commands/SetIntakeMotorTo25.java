package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetIntakeMotorTo25 extends Command {

	private double leftPercent = -25;
	private double rightPercent = -25;
	
    public SetIntakeMotorTo25() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setIntakeMotorToPercentPower(leftPercent, rightPercent);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.intake.setIntakeMotorToPercentPower(leftPercent, rightPercent);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	if (Robot.intake.readLeftIntakeMotor() != -25 && Robot.intake.readRightIntakeMotor() != -25) {
    		return false;
    	}
    	
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
