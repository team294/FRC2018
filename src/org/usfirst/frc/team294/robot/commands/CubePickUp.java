package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
//import java.util.concurrent.*;

/**
 *
 */
public class CubePickUp extends Command {

	private double percent;
	
    public CubePickUp(double percent) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.claw);
    	this.percent = percent;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.claw.setClawMotorToPercentPower(percent);
    	Robot.claw.extendPiston();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.inputs.isObjectPresent()) {
    		Robot.claw.retractPiston();
    		//wait(250);
    		Robot.claw.setClawMotorToPercentPower(0);
    	}
    	
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
