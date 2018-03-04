package org.usfirst.frc.team294.robot.commands;

import org.usfirst.frc.team294.robot.Robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class overrideArmControl extends Command {
	
	private boolean joystickToggle = false;
    
	public overrideArmControl() {
        requires(Robot.armMotor);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	if(joystickToggle) {
    		joystickToggle = false;
    	}
    	else {
    		joystickToggle =  true;
    	}
    	if( joystickToggle) {
    		Robot.armMotor.joystickControl = true;
    	}
    	else {
    		Robot.armMotor.joystickControl = false;
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	
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
