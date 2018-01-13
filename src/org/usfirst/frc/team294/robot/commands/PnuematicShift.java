package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.OI;
import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PnuematicShift extends Command {

    public PnuematicShift() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.shifterSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if(OI.leftJoystick.getRawButton(4)) {
    	Robot.shifterSubsystem.setShift(true);
    	}else 
        	if(OI.leftJoystick.getRawButton(5)) {
            	Robot.shifterSubsystem.setShift(false);
            	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.shifterSubsystem.shiftDown();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
